# This script is intentionally parser-lite: it keeps original Intel HEX records,
# validates that every data record belongs to the expected flash region, then
# writes bootloader records + app records + one final EOF record.

# CMake -P has no target context, so all paths and flash boundaries are passed
# explicitly from the root CMakeLists.txt custom command.
foreach(required_var BOOT_HEX APP_HEX OUT_HEX BOOT_START BOOT_END APP_START APP_END)
    if(NOT DEFINED ${required_var})
        message(FATAL_ERROR "${required_var} is not provided")
    endif()
endforeach()

# Fail early if the build did not produce one of the two input images.
foreach(input_hex BOOT_HEX APP_HEX)
    if(NOT EXISTS "${${input_hex}}")
        message(FATAL_ERROR "${input_hex} does not exist: ${${input_hex}}")
    endif()
endforeach()

# Convert hex-looking cache values such as 0x08003000 into numbers that CMake
# can compare in range checks below.
math(EXPR BOOT_START_DEC "${BOOT_START}")
math(EXPR BOOT_END_DEC "${BOOT_END}")
math(EXPR APP_START_DEC "${APP_START}")
math(EXPR APP_END_DEC "${APP_END}")

# The full image assumes a simple adjacent layout:
# [bootloader region) immediately followed by [application region).
if(NOT BOOT_END_DEC EQUAL APP_START_DEC)
    message(FATAL_ERROR "Bootloader end must match application start")
endif()

# Read an Intel HEX file and return all records except EOF. Data records are
# checked against the allowed flash range; type 04 records update the address
# base; type 05 start records are kept only for the bootloader image.
function(collect_ihex_records file label min_addr max_addr keep_start_record out_records out_bytes out_min)
    file(READ "${file}" contents)
    string(REPLACE "\r\n" "\n" contents "${contents}")
    string(REPLACE "\r" "\n" contents "${contents}")
    string(REPLACE "\n" ";" lines "${contents}")

    set(records)
    set(upper_addr 0)
    set(total_bytes 0)
    set(first_addr "")
    set(eof_seen FALSE)

    foreach(line IN LISTS lines)
        string(STRIP "${line}" line)
        if("${line}" STREQUAL "")
            continue()
        endif()

        string(LENGTH "${line}" line_len)
        if(line_len LESS 11)
            message(FATAL_ERROR "${label}: Intel HEX line is too short: ${line}")
        endif()

        string(SUBSTRING "${line}" 0 1 marker)
        if(NOT marker STREQUAL ":")
            message(FATAL_ERROR "${label}: invalid Intel HEX line: ${line}")
        endif()

        string(SUBSTRING "${line}" 1 2 len_hex)
        string(SUBSTRING "${line}" 3 4 addr_hex)
        string(SUBSTRING "${line}" 7 2 type_hex)
        math(EXPR byte_count "0x${len_hex}")
        math(EXPR record_addr "0x${addr_hex}")
        math(EXPR record_type "0x${type_hex}")

        # Type 00 contains bytes programmed into flash. The 16-bit record
        # address is relative to the latest type 04 upper address.
        if(record_type EQUAL 0)
            if(byte_count GREATER 0)
                math(EXPR abs_start "${upper_addr} + ${record_addr}")
                math(EXPR abs_end "${abs_start} + ${byte_count}")
                if(abs_start LESS min_addr OR abs_end GREATER max_addr)
                    message(FATAL_ERROR
                        "${label}: data record 0x${addr_hex} is outside allowed range "
                        "[${min_addr}, ${max_addr})"
                    )
                endif()
                if("${first_addr}" STREQUAL "" OR abs_start LESS first_addr)
                    set(first_addr ${abs_start})
                endif()
                math(EXPR total_bytes "${total_bytes} + ${byte_count}")
            endif()
            list(APPEND records "${line}")
        # Drop input EOF records. The merged file must have exactly one EOF,
        # which is appended after both images have been concatenated.
        elseif(record_type EQUAL 1)
            if(eof_seen)
                message(FATAL_ERROR "${label}: multiple EOF records")
            endif()
            set(eof_seen TRUE)
        # Type 04 selects the upper 16 address bits for following data records.
        elseif(record_type EQUAL 4)
            if(NOT byte_count EQUAL 2)
                message(FATAL_ERROR "${label}: invalid extended linear address record: ${line}")
            endif()
            math(EXPR data_chars "${byte_count} * 2")
            string(SUBSTRING "${line}" 9 ${data_chars} data_hex)
            math(EXPR upper_addr "0x${data_hex} << 16")
            list(APPEND records "${line}")
        # Type 05 is the start execution address. The final image should keep
        # the bootloader entry point and ignore the app one.
        elseif(record_type EQUAL 5)
            if(keep_start_record)
                list(APPEND records "${line}")
            endif()
        else()
            message(FATAL_ERROR "${label}: unsupported Intel HEX record type ${type_hex}")
        endif()
    endforeach()

    if(NOT eof_seen)
        message(FATAL_ERROR "${label}: missing EOF record")
    endif()
    if(total_bytes EQUAL 0)
        message(FATAL_ERROR "${label}: no data records found in ${file}")
    endif()

    set(${out_records} ${records} PARENT_SCOPE)
    set(${out_bytes} ${total_bytes} PARENT_SCOPE)
    set(${out_min} ${first_addr} PARENT_SCOPE)
endfunction()

# Parse both images with their own allowed ranges. This catches accidental
# linker-script drift before producing a file that CubeProgrammer would accept.
collect_ihex_records("${BOOT_HEX}" "VBBoot" ${BOOT_START_DEC} ${BOOT_END_DEC} TRUE boot_records boot_bytes boot_min)
collect_ihex_records("${APP_HEX}" "VBDrive" ${APP_START_DEC} ${APP_END_DEC} FALSE app_records app_bytes app_min)

# Ensure each image starts at the expected flash base, not merely somewhere
# inside the allowed region.
if(NOT boot_min EQUAL BOOT_START_DEC)
    message(FATAL_ERROR "VBBoot hex must start at ${BOOT_START}, got ${boot_min}")
endif()
if(NOT app_min EQUAL APP_START_DEC)
    message(FATAL_ERROR "VBDrive hex must start at ${APP_START}, got ${app_min}")
endif()

# Preserve the original records and append a single EOF record for the merged
# image. No checksum rewriting is needed because records are not modified.
set(merged_records ${boot_records} ${app_records} ":00000001FF")
string(REPLACE ";" "\n" merged_text "${merged_records}")
file(WRITE "${OUT_HEX}" "${merged_text}\n")

message(STATUS "VBBoot bytes: ${boot_bytes}")
message(STATUS "VBDrive bytes: ${app_bytes}")
message(STATUS "Merged full image: ${OUT_HEX}")
