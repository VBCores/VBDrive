#!/usr/bin/env python3
"""Numerical and lifecycle tests of the standalone trajectory generators."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
program = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include <type_traits>
#include "voltbro/motors/trajectories/filter.hpp"
#include "voltbro/motors/trajectories/poly.hpp"
#include "voltbro/motors/trajectories/ramp.hpp"

void near(float a,float b,float tolerance=2e-3f) {
    if (!(std::isfinite(a) && std::fabs(a-b)<tolerance)) std::fprintf(stderr,"%f != %f\n",a,b);
    assert(std::isfinite(a) && std::fabs(a-b)<tolerance);
}
void trajectory(float p,float v,float goal,float speed,float accel,float decel) {
    PolyTrajectory poly(speed,accel,decel);
    TrajectoryGenerator& input=poly;
    assert(input.start({p,v},goal));
    near(poly.reference,p,1e-5f);near(poly.velocity,v,1e-5f);
    float previous=poly.velocity;
    for(int tick=0;tick<300000;++tick) {
        input.step(.0001f);
        assert(std::fabs(poly.velocity)<=std::max(speed,std::fabs(v))+.01f);
        if(poly.elapsed<poly.finish_time) {
            float observed=std::fabs(poly.velocity-previous)/.0001f;
            assert(observed<=std::max(accel,decel)+.03f);
            if(poly.velocity*previous>=0 && std::fabs(poly.velocity)<std::fabs(previous))
                assert(observed<=decel+.03f);
        }
        previous=poly.velocity;
        if(poly.elapsed>=poly.finish_time)break;
    }
    assert(poly.reference==goal && poly.velocity==0);
    input.step(.0001f);assert(poly.reference==goal);
    float elapsed=poly.elapsed;
    assert(!input.retarget({0,0},NAN));assert(poly.elapsed==elapsed && poly.reference==goal);
    input.reset();assert(!input.retarget({0,0},goal));
    assert(input.start({p,v},goal));
    input.step(.001f);float sampled=poly.reference;
    near(poly.sample_at(.001f),sampled,1e-6f);
}
int main() {
    static_assert(std::is_abstract_v<TrajectoryGenerator>);
    RampTrajectory ramp;
    TrajectoryGenerator& r=ramp;
    assert(!r.start({0,0},1));assert(ramp.configure(2));
    assert(r.start({0,.2f},1));near(r.step(.1f),.4f,1e-6f);
    assert(r.retarget({100,100},1));near(r.step(.1f),.6f,1e-6f);
    assert(r.retarget({100,100},-1));near(r.step(.1f),.4f,1e-6f);
    assert(!r.retarget({0,0},INFINITY));near(ramp.goal,-1);
    assert(ramp.configure(1));near(r.step(.1f),.3f,1e-6f);
    for(int i=0;i<30;++i)r.step(.1f);
    assert(ramp.reference==-1);
    r.reset();assert(!r.retarget({0,0},0));
    assert(r.start({0,0},-.5f));near(r.step(.1f),-.1f,1e-6f);
    float ref=ramp.reference;assert(!ramp.configure(INFINITY));
    assert(!r.start({0,0},NAN));assert(ramp.reference==ref);

    FilterTrajectory filter(10);
    TrajectoryGenerator& f=filter;
    assert(f.start({0,.2f},1));near(f.step(.001f),.000296f,1e-6f);
    float earlier=filter.reference;
    assert(f.retarget({100,100},2));assert(filter.reference==earlier);
    for(int i=0;i<2000;++i)f.step(.001f);
    near(filter.reference,2,.001f);
    assert(f.retarget({100,100},-1));
    for(int i=0;i<2000;++i)f.step(.001f);
    near(filter.reference,-1,.001f);
    earlier=filter.reference;
    assert(!f.start({NAN,0},2) && filter.reference==earlier);
    assert(!f.retarget({0,0},NAN));
    assert(filter.configure(1));f.retarget({0,0},2);assert(filter.reference==earlier);
    f.reset();assert(!f.retarget({0,0},2));
    assert(f.start({0,0},1));assert(filter.configure(1e6f));
    near(f.step(.001f),.0625f,1e-6f); // Effective bandwidth 0.25/dt.

    trajectory(0,0,5,2,3,4);
    trajectory(0,0,.1f,2,3,4);
    trajectory(1,0,1,2,3,4);
    trajectory(0,-.5f,2,2,3,4);
    trajectory(0,3,5,2,3,4);
    trajectory(0,2,.1f,2,3,4);
    trajectory(0,-2,-.1f,2,3,4);
    trajectory(0,3,20,2,8,.5f);
    trajectory(0,3,.1f,2,8,.5f);
    PolyTrajectory poly(2,3,4);TrajectoryGenerator& p=poly;
    assert(p.start({0,0},3));p.step(.1f);
    assert(p.retarget({1,.1f},4));assert(poly.reference==1 && poly.elapsed==0);
    assert(poly.configure(1,3,4));assert(p.retarget({2,0},4));assert(poly.reference==2);
    float goal=poly.goal;assert(!poly.configure(1,3,0));
    assert(!p.retarget({0,0},NAN));assert(poly.goal==goal && poly.reference==2);
    PolyTrajectory long_run(1,1,1); TrajectoryGenerator& clocked=long_run;
    assert(clocked.start({0,0},1000));
    for(int tick=0;tick<300000;++tick) clocked.step(.0002f);
    near(long_run.elapsed,60,1e-5f);
    const float advanced=long_run.reference;
    near(long_run.sample_at(60),advanced,1e-5f);
    clocked.reset(); assert(clocked.start({0,0},1000));
    near(clocked.step(.1f),.005f,1e-7f);
    // The same publication/activation lifecycle applies through the base interface.
    FilterTrajectory old_filter(10), new_filter(20);
    assert(old_filter.start({0,.2f},1)); old_filter.step(.01f);
    TrajectoryGenerator& prepared_filter=new_filter;
    assert(prepared_filter.start({99,99},2));
    prepared_filter.on_publish(&old_filter,100);
    near(new_filter.reference,old_filter.reference,1e-7f);
    near(new_filter.velocity,old_filter.velocity,1e-7f);
    assert(new_filter.goal==2);
    prepared_filter.on_activate({.3f,.4f});
    near(new_filter.reference,.3f,1e-7f); near(new_filter.velocity,.4f,1e-7f);
    near(prepared_filter.step(.001f),.301064f,1e-6f); // New bandwidth survives publication.
    RampTrajectory old_ramp(1), new_ramp(2);
    assert(old_ramp.start({0,.2f},1)); old_ramp.step(.01f);
    TrajectoryGenerator& prepared_ramp=new_ramp;
    assert(prepared_ramp.start({99,99},2));
    prepared_ramp.on_publish(&old_ramp,100);
    near(new_ramp.reference,old_ramp.reference,1e-7f); assert(new_ramp.goal==2);
    prepared_ramp.on_activate({99,.4f}); near(new_ramp.reference,.4f,1e-7f);
    near(prepared_ramp.step(.1f),.6f,1e-7f); // New slew rate survives publication.
    PolyTrajectory published(1,2,2); TrajectoryGenerator& prepared_poly=published;
    assert(prepared_poly.start({0,0},1));
    prepared_poly.on_publish(nullptr,.001f);
    prepared_poly.on_activate({99,99}); // Prepared POLY initial conditions remain intact.
    near(prepared_poly.step(.0002f),.0012f*.0012f,1e-8f);
    puts("PASS: standalone/base-reference lifecycle, limits, filter/ramp and polynomial profiles");
}
'''
with tempfile.TemporaryDirectory(prefix='vbdrive-trajectories-') as directory:
    path=Path(directory)
    (path/'test.cpp').write_text(program)
    subprocess.run(['c++','-std=c++20','-O2','-I'+str(root/'Drivers/libvoltbro'),
                    str(path/'test.cpp'),'-o',str(path/'test')],check=True)
    subprocess.run([str(path/'test')],check=True)
