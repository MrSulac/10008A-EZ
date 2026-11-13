#include <cassert>
#include <climits>
#include <iostream>
#include <random>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <chrono>
#include "../../src/ekf.cpp"
#include "../../src/motion_profile.cpp"
#include "../../src/mcl.cpp"
#include "../../src/autons/profile-test.cpp"
#include "../../src/stanley.cpp"
#include "../../src/autons/stanley-test.cpp"
#include "../../src/ramsete.cpp"
#include "../../src/autons/ramsete-test.cpp"

char traceDirectory[256]={};
#define AUTON_TRACE_DIRECTORY traceDirectory
#include "../../src/auton_trace.cpp"

constexpr double nominalFreeSpeed=3.14159265358979323846*3.25*360.0/60.0;
constexpr double rad=3.14159265358979323846/180;
void near(double actual,double expected,double tol){if(!std::isfinite(actual)||std::abs(actual-expected)>tol){std::cerr<<"Expected "<<expected<<" got "<<actual<<" tol "<<tol<<'\n';std::abort();}}
void pass(const char* name){std::cout<<"PASS "<<name<<'\n';}
void psd(const ekf::Matrix& p){
 for(int i=0;i<3;++i){assert(p[i][i]>=-1e-10);for(int j=0;j<3;++j)near(p[i][j],p[j][i],1e-9);}
 for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)assert(p[i][i]*p[j][j]-p[i][j]*p[j][i]>=-1e-9);
 double det=p[0][0]*(p[1][1]*p[2][2]-p[1][2]*p[2][1])-p[0][1]*(p[1][0]*p[2][2]-p[1][2]*p[2][0])+p[0][2]*(p[1][0]*p[2][1]-p[1][1]*p[2][0]);assert(det>=-1e-9);
}
// Independent geometric intersection oracle, also handles corner-origin rays.
void observe(ekf::Pose truth){
 Sensor* configured[]={&frontSensor,&backSensor,&leftSensor,&rightSensor};
 for(auto source:configured){
  const double a=truth.theta*rad,ox=truth.x+std::cos(a)*source->offset.x+std::sin(a)*source->offset.y,oy=truth.y-std::sin(a)*source->offset.x+std::cos(a)*source->offset.y;
  const double dx=std::sin(a+source->angle*rad),dy=std::cos(a+source->angle*rad);double nearest=1e9;
  if(std::abs(dx)>1e-8) for(double x:{0.,144.}){double t=(x-ox)/dx,y=oy+t*dy;if(t>0&&y>=0&&y<=144)nearest=std::min(nearest,t);}
  if(std::abs(dy)>1e-8) for(double y:{0.,144.}){double t=(y-oy)/dy,x=ox+t*dx;if(t>0&&x>=0&&x<=144)nearest=std::min(nearest,t);}
  source->distanceSensor.value=nearest*25.4<=2000?std::lround(nearest*25.4):9999;
 }
}
void setup(ekf::Pose pose){
 pros::tick={};pros::clockMs+=1000;pros::competition::connected=false;chassis=ez::Drive{};
 initializeLocalizationMap();ekf::reset(pose);observe(pose);pros::clockMs+=20;ekf::update();
}
int main(){
 const auto traceTemp=std::filesystem::temp_directory_path()/("auton-trace-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
 assert(std::filesystem::create_directory(traceTemp));std::snprintf(traceDirectory,sizeof(traceDirectory),"%s",traceTemp.string().c_str());
 ekf::Filter f;f.reset({40,40,0},0,0);f.predict(10,10);near(f.pose().x,40,1e-10);near(f.pose().y,50,1e-10);
 f.predict(3.141592653589793*3,-3.141592653589793*3);near(f.pose().theta,90,1e-10);f.predict(10,10);near(f.pose().x,50,1e-10);
 f.reset({0,0,0},0,0);f.predict(12*rad,0);near(f.pose().theta,1,1e-10);psd(f.covariance());pass("encoder prediction, clockwise turns, arc integration");
 ekf::SensorMount mount{3,5.75,0};ekf::Wall wall{0,144,144,144};ekf::Pose pose{50,60,23};
 f.reset(pose);const auto range=f.rangePrediction(mount,wall);assert(range.valid);
 for(int j=0;j<3;++j){auto plus=pose,minus=pose;double step=1e-5;
  if(j==0){plus.x+=step;minus.x-=step;}if(j==1){plus.y+=step;minus.y-=step;}if(j==2){plus.theta+=step/rad;minus.theta-=step/rad;}
  f.reset(plus);auto rplus=f.rangePrediction(mount,wall);f.reset(minus);auto rminus=f.rangePrediction(mount,wall);
  near(range.jacobian[j],(rplus.distance-rminus.distance)/(2*step),1e-6);
 }
 pass("analytic distance Jacobian matches finite differences");
 f.reset(pose);auto before=f.pose();assert(!f.correctRange(range.distance-50,mount,wall));near(f.pose().y,before.y,0);assert(!f.correctRange(-1,mount,wall));
 assert(!f.rangePrediction({0,0,90},{0,144,144,144}).valid);pass("innovation gating, bad measurements, grazing and missed walls");
 f.reset({50,40,0});ekf::Filter truth;truth.reset({50,40,0});
 const ekf::Wall back{0,0,144,0},left{0,144,0,0};
 for(int i=0;i<400;++i){truth.predict(.05,.05);f.predict(.06,.06);if(i%3==0){
  auto a=truth.rangePrediction({0,-5.75,180},back);auto b=truth.rangePrediction({-5.75,0,270},left);
  if(a.valid)f.correctRange(a.distance,{0,-5.75,180},back);if(b.valid)f.correctRange(b.distance,{-5.75,0,270},left);
 }psd(f.covariance());}
 std::cout<<"  4-inch encoder drift -> EKF error "<<std::hypot(f.pose().x-truth.pose().x,f.pose().y-truth.pose().y)<<" in\n";
 assert(std::hypot(f.pose().x-truth.pose().x,f.pose().y-truth.pose().y)<1.5);pass("encoder drift correction and positive covariance");
 std::mt19937 random(17);std::normal_distribution<double> n(0,1);f.reset({72,72,0});
 for(int i=0;i<10000;++i){f.predict(.01*n(random),.01*n(random));auto r=f.rangePrediction({0,5.75,0},wall);if(r.valid)f.correctRange(r.distance+.5*n(random),{0,5.75,0},wall);psd(f.covariance());}
 pass("10000 noisy Joseph-form covariance updates");
 motion::Path path;assert(path.build({{0,0,0},{0,24,0},{24,48,90},{48,48,90}}));
 near(path.sample(0).velocity,0,0);near(path.sample(path.duration()).velocity,0,0);near(path.sample(path.duration()).pose.x,48,1e-10);near(path.sample(path.duration()).pose.theta,90,1e-8);
 const auto limits=path.limits();
 for(std::size_t i=1;i<path.size();++i){const auto&a=path.node(i-1);const auto&b=path.node(i);double ds=b.distance-a.distance,ac=(b.velocity*b.velocity-a.velocity*a.velocity)/(2*ds);assert(b.time>a.time);assert(std::abs(ac)<=limits.maxAcceleration+1e-7);
  for(double sign:{-1.,1.}){double qa=1+sign*limits.trackWidth*.5*a.curvature,qb=1+sign*limits.trackWidth*.5*b.curvature,derivative=(qb-qa)/ds;assert(std::abs(qa*ac+derivative*a.velocity*a.velocity)<=limits.maxWheelAcceleration+1e-6);assert(std::abs(qb*ac+derivative*b.velocity*b.velocity)<=limits.maxWheelAcceleration+1e-6);}
 }
 for(double t=0;t<path.duration();t+=.001){auto s=path.sample(t);assert(s.velocity<=limits.maxVelocity+1e-6);assert(s.velocity*(1+std::abs(s.curvature)*limits.trackWidth*.5)<=limits.maxWheelVelocity+1e-5);assert(std::abs(s.angularVelocity)<=limits.maxAngularVelocity+1e-5);assert(s.velocity*s.velocity*std::abs(s.curvature)<=limits.maxLateralAcceleration+1e-5);}
 std::cout<<"  Path "<<path.length()<<" in, "<<path.duration()<<" s, "<<path.size()<<" samples\n";
 pass("2D profile endpoints, speed, center/wheel acceleration, yaw limits");
 assert(!path.build({{0,0,0},{0,0,90}}));assert(!path.valid());motion::Limits invalid;invalid.maxAcceleration=0;assert(!path.build({{0,0,0},{0,24,0}},invalid));assert(!path.build({{0,0,0},{0,2000,0}}));
 auto command=motion::feedback({0,0,0},{{1,10,0},10,0,0,0,10});assert(command.forward>10&&command.angular>0);pass("invalid profiles rejected and feedback steers toward path");
 setup({50,60,0});assert(ekf::getEstimate().healthy);assert(ekf::getEstimate().acceptedRanges>=2);
 mcl::reset({50,60,0});assert(!ekf::getEstimate().enabled);assert(mcl::getComparison().enabled);
 ekf::reset({50,60,0});assert(!mcl::getComparison().enabled);pass("MCL and EKF ownership switches are exclusive");
 setup({50,60,0});const double beforeBad=ekf::getEstimate().pose.y;frontDistance.value=INT_MAX;backDistance.value=9999;leftDistance.value=0;rightDistance.value=-1;
 for(int i=0;i<4;++i){chassis.left+=.1;chassis.right+=.1;pros::clockMs+=20;ekf::update();}
 near(ekf::getEstimate().pose.y,beforeBad+.4,.001);assert(ekf::getEstimate().acceptedRanges==0);pass("runtime invalid ranges use encoder prediction");
 // Closed-loop simulated motor dynamics, sensor readings, EKF and actual follow().
 setup({0,0,0});motion::Limits testLimits;testLimits.maxVelocity=18;testLimits.maxAcceleration=24;testLimits.maxWheelVelocity=24;testLimits.maxWheelAcceleration=36;assert(path.build({{0,0,0},{0,12,0},{12,24,90},{24,24,90}},testLimits));
 const auto routeStarted=pros::millis();
 ekf::Pose actual{0,0,0};double vl=0,vr=0;
 pros::tick=[&]{
  auto target=[](int c){return std::abs(c)>5?(c-std::copysign(5.,c))/(127./nominalFreeSpeed):0.;};
  vl+=(target(chassis.leftCommand)-vl)*.28;vr+=(target(chassis.rightCommand)-vr)*.28;
  double dl=vl*.02,dr=vr*.02,turn=(dl-dr)/12,theta=actual.theta*rad+turn*.5,d=(dl+dr)*.5;
  if(std::abs(turn)>1e-9)d*=std::sin(turn*.5)/(turn*.5);
  actual.x+=d*std::sin(theta);actual.y+=d*std::cos(theta);actual.theta=std::remainder(actual.theta+turn/rad,360.);
  chassis.left+=dl;chassis.right+=dr;observe(actual);ekf::update();auton_trace::service();
 };
 ekf_profile_test(); auto result=ez::lastScreen.find("Profile settled:")==0 ? motion::Result::Settled : motion::Result::Timeout;std::cout<<"  Full follower: "<<motion::resultName(result)<<" at "<<actual.x<<","<<actual.y<<","<<actual.theta<<" in "<<(pros::millis()-routeStarted)<<" ms\n";
 assert(result==motion::Result::Settled);assert(std::hypot(actual.x-24,actual.y-24)<1.2);assert(std::abs(actual.theta-90)<4);assert(chassis.leftCommand==0&&chassis.rightCommand==0);pass("complete test route with motor dynamics and sensor fusion");
 // Ramsete equation checks use known body-frame errors and exact references.
 motion::Sample ramRef;ramRef.pose={5,10,0};ramRef.velocity=10;
 auto ramCommand=ramsete::feedback({0,0,0},ramRef);
 near(ramCommand.forward,10+10*1.7146428199482244,1e-10);near(ramCommand.angular,.75,1e-12);
 ramRef.pose={0,10,0};const auto north=ramsete::feedback({0,0,0},ramRef);
 ramRef.pose={10,0,90};const auto east=ramsete::feedback({0,0,90},ramRef);
 near(east.forward,north.forward,1e-10);near(east.angular,north.angular,1e-10);
 ramRef.pose={4,10,90};ramRef.velocity=8;ramRef.angularVelocity=.5;
 ramCommand=ramsete::feedback(ramRef.pose,ramRef);near(ramCommand.forward,8,1e-12);near(ramCommand.angular,.5,1e-12);
 ramRef.pose={0,0,0};ramRef.velocity=10;ramRef.angularVelocity=0;
 assert(ramsete::feedback({2,0,0},ramRef).angular<0);assert(ramsete::feedback({-2,0,0},ramRef).angular>0);
 ramRef.pose.theta=1;ramCommand=ramsete::feedback({0,0,359},ramRef);assert(ramCommand.angular>0&&ramCommand.angular<.2);
 ramRef.pose.theta=0;ramCommand=ramsete::feedback({0,0,1e-10},ramRef);assert(std::isfinite(ramCommand.angular));
 ramRef.pose={0,10,0};ramsete::Config stronger;stronger.b=.03;
 assert(ramsete::feedback({0,0,0},ramRef,stronger).forward>ramsete::feedback({0,0,0},ramRef).forward);
 ramRef.velocity=0;ramCommand=ramsete::feedback({2,0,30},ramRef);near(ramCommand.forward,0,0);near(ramCommand.angular,0,0);
 pass("Ramsete equations, frame rotation, curvature, steering signs, wrap and configurable gains");
 // Run the actual selectable Ramsete test with simulated motor dynamics.
 setup({0,0,0});actual={0,0,0};vl=vr=0;const auto ramseteStarted=pros::millis();
 pros::usd::installed=true;assert(auton_trace::begin("Ramsete Profile"));bool observedDuringMatch=false;
 pros::tick=[&]{
  auto target=[](int c){return std::abs(c)>5?(c-std::copysign(5.,c))/(127./nominalFreeSpeed):0.;};
  vl+=(target(chassis.leftCommand)-vl)*.28;vr+=(target(chassis.rightCommand)-vr)*.28;
  double dl=vl*.02,dr=vr*.02,turn=(dl-dr)/12,theta=actual.theta*rad+turn*.5,d=(dl+dr)*.5;
  if(std::abs(turn)>1e-9)d*=std::sin(turn*.5)/(turn*.5);
  actual.x+=d*std::sin(theta);actual.y+=d*std::cos(theta);actual.theta=std::remainder(actual.theta+turn/rad,360.);
  chassis.left+=dl;chassis.right+=dr;observe(actual);ekf::update();auton_trace::service();
 };
 ramsete_profile_test();
 {std::ifstream file(auton_trace::status().file);std::string text((std::istreambuf_iterator<char>(file)),{});observedDuringMatch=text.find("t,x,y,theta")!=std::string::npos&&text.size()>100;}
 assert(observedDuringMatch&&auton_trace::status().recording);
 const std::string autonLog=auton_trace::status().file;auton_trace::end();
 assert(auton_trace::begin("Driver-Control"));auton_trace::service();
 assert(auton_trace::status().filesSaved>=1);assert(std::string(auton_trace::status().file)!=autonLog);
 auton_trace::end();auton_trace::service();
 {std::ifstream file(autonLog);std::string text((std::istreambuf_iterator<char>(file)),{});assert(text.find(",ekf")!=std::string::npos);if(const auto dest=std::getenv("AUTON_TRACE_TEST_OUTPUT")){std::ofstream out(dest);out<<text;}}
 pass("position CSV is flushed during a match and auton/driver logs stay separate");
 std::cout<<"  Ramsete at "<<actual.x<<","<<actual.y<<","<<actual.theta<<" in "<<(pros::millis()-ramseteStarted)<<" ms\n";
 assert(ez::lastScreen.find("Ramsete settled:")==0);assert(std::hypot(actual.x-24,actual.y-24)<1.2);assert(std::abs(actual.theta-90)<4);
 assert(chassis.leftCommand==0&&chassis.rightCommand==0);pass("actual Ramsete autonomous with motors, encoders and distance sensors");
 // Correct a starting pose offset using independently configurable gains.
 assert(path.build({{40,40,0},{40,52,0},{52,64,90},{64,64,90}},testLimits));
 setup({42,40,8});actual={42,40,8};vl=vr=0;
 pros::tick=[&]{
  auto target=[](int c){return std::abs(c)>5?(c-std::copysign(5.,c))/(127./nominalFreeSpeed):0.;};
  vl+=(target(chassis.leftCommand)-vl)*.28;vr+=(target(chassis.rightCommand)-vr)*.28;
  double dl=vl*.02,dr=vr*.02,turn=(dl-dr)/12,theta=actual.theta*rad+turn*.5,d=(dl+dr)*.5;
  if(std::abs(turn)>1e-9)d*=std::sin(turn*.5)/(turn*.5);
  actual.x+=d*std::sin(theta);actual.y+=d*std::cos(theta);actual.theta=std::remainder(actual.theta+turn/rad,360.);
  chassis.left+=dl;chassis.right+=dr;observe(actual);ekf::update();auton_trace::service();
 };
 assert(ramsete::follow(path,{.02,.75},{},14000)==motion::Result::Settled);
 assert(std::hypot(actual.x-64,actual.y-64)<1.2);assert(std::abs(actual.theta-90)<4);
 pass("Ramsete recovers from lateral/heading offsets with nondefault gains");
 assert(path.build({{0,0,0},{0,12,0},{12,24,90},{24,24,90}},testLimits));
 setup({0,0,0});const auto ramCancel=pros::millis()+100;pros::tick=[&]{ekf::update();if(pros::millis()>=ramCancel)motion::cancel();};
 assert(ramsete::follow(path)==motion::Result::Cancelled);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 setup({0,0,0});pros::tick=[]{};assert(ramsete::follow(path)==motion::Result::LocalizationUnavailable);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 setup({0,0,0});pros::tick=[]{ekf::update();};assert(ramsete::follow(path,{},{},500)==motion::Result::Timeout);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 setup({0,0,0});pros::competition::connected=true;pros::competition::disabled=true;assert(ramsete::follow(path)==motion::Result::Cancelled);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 assert(ramsete::follow(path,{0,.7})==motion::Result::InvalidPath);assert(ramsete::follow(path,{.015,1})==motion::Result::InvalidPath);
 motion::Path emptyPath;assert(ramsete::follow(emptyPath)==motion::Result::InvalidPath);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 pass("Ramsete cancel, stale EKF, timeout, disable, invalid gains and empty paths stop motors");
 // Stanley signs and finite low-speed behavior in the clockwise field frame.
 motion::Sample line;line.pose={0,10,0};line.velocity=10;
 auto steer=stanley::feedback({2,10,0},line,0);assert(steer.angular<0);assert(std::isfinite(steer.angular));
 steer=stanley::feedback({-2,10,0},line,0);assert(steer.angular>0);
 steer=stanley::feedback({0,10,10},line,10);assert(steer.angular<0);
 steer=stanley::feedback({0,10,0},line,10);near(steer.forward,10,1e-12);near(steer.angular,0,1e-12);
 line.curvature=.05;steer=stanley::feedback({0,10,0},line,10);near(steer.angular,.5,1e-12);
 line.curvature=0;line.pose.theta=1;steer=stanley::feedback({0,10,359},line,0);assert(steer.angular>0&&steer.angular<.2);
 pass("Stanley left/right signs, heading wrap, rest and curvature feedforward");
 // Validate the actual selectable Stanley autonomous routine with the same plant.
 setup({0,0,0});actual={0,0,0};vl=vr=0;const auto stanleyStarted=pros::millis();
 pros::tick=[&]{
  auto target=[](int c){return std::abs(c)>5?(c-std::copysign(5.,c))/(127./nominalFreeSpeed):0.;};
  vl+=(target(chassis.leftCommand)-vl)*.28;vr+=(target(chassis.rightCommand)-vr)*.28;
  double dl=vl*.02,dr=vr*.02,turn=(dl-dr)/12,theta=actual.theta*rad+turn*.5,d=(dl+dr)*.5;
  if(std::abs(turn)>1e-9)d*=std::sin(turn*.5)/(turn*.5);
  actual.x+=d*std::sin(theta);actual.y+=d*std::cos(theta);actual.theta=std::remainder(actual.theta+turn/rad,360.);
  chassis.left+=dl;chassis.right+=dr;observe(actual);ekf::update();auton_trace::service();
 };
 stanley_profile_test();std::cout<<"  Stanley at "<<actual.x<<","<<actual.y<<","<<actual.theta<<" in "<<(pros::millis()-stanleyStarted)<<" ms\n";
 assert(ez::lastScreen.find("Stanley settled:")==0);assert(std::hypot(actual.x-24,actual.y-24)<1.2);assert(std::abs(actual.theta-90)<4);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 pass("actual Stanley autonomous with motor dynamics, encoders and distance sensors");
 // Recover from a known lateral and heading offset on an interior route.
 assert(path.build({{40,40,0},{40,52,0},{52,64,90},{64,64,90}},testLimits));
 setup({42,40,8});actual={42,40,8};vl=vr=0;
 pros::tick=[&]{
  auto target=[](int c){return std::abs(c)>5?(c-std::copysign(5.,c))/(127./nominalFreeSpeed):0.;};
  vl+=(target(chassis.leftCommand)-vl)*.28;vr+=(target(chassis.rightCommand)-vr)*.28;
  double dl=vl*.02,dr=vr*.02,turn=(dl-dr)/12,theta=actual.theta*rad+turn*.5,d=(dl+dr)*.5;
  if(std::abs(turn)>1e-9)d*=std::sin(turn*.5)/(turn*.5);
  actual.x+=d*std::sin(theta);actual.y+=d*std::cos(theta);actual.theta=std::remainder(actual.theta+turn/rad,360.);
  chassis.left+=dl;chassis.right+=dr;observe(actual);ekf::update();auton_trace::service();
 };
 assert(stanley::follow(path,{}, {},14000)==motion::Result::Settled);
 assert(std::hypot(actual.x-64,actual.y-64)<1.2);assert(std::abs(actual.theta-90)<4);
 assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 pass("Stanley recovers from 2-inch lateral and 8-degree heading errors");
 assert(path.build({{0,0,0},{0,12,0},{12,24,90},{24,24,90}},testLimits));
 // Spatial projection handles lateral/heading errors and never jumps backward.
 stanley::Tracker tracker{};auto command1=stanley::track(path,{-2,5,0},{},3,&tracker);
 assert(command1.velocity.angular>0);const double progress=tracker.progress;
 stanley::track(path,{-2,4,0},{},3,&tracker);assert(tracker.progress>=progress);
 pass("Stanley projected cross-track correction and monotonic progress");
 setup({0,0,0});const auto stanleyCancel=pros::millis()+100;pros::tick=[&]{ekf::update();if(pros::millis()>=stanleyCancel)motion::cancel();};
 assert(stanley::follow(path)==motion::Result::Cancelled);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 setup({0,0,0});pros::tick=[]{};assert(stanley::follow(path)==motion::Result::LocalizationUnavailable);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 setup({0,0,0});pros::tick=[]{ekf::update();};assert(stanley::follow(path,{}, {},500)==motion::Result::Timeout);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 setup({0,0,0});pros::competition::connected=true;pros::competition::disabled=true;assert(stanley::follow(path)==motion::Result::Cancelled);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 stanley::Config bad;bad.softeningSpeed=0;assert(stanley::follow(path,bad)==motion::Result::InvalidPath);assert(chassis.leftCommand==0&&chassis.rightCommand==0);
 pass("Stanley cancel, stale state, timeout, disable and invalid configuration stop motors");
 setup({0,0,0});const auto cancelTime=pros::millis()+100;pros::tick=[&]{ekf::update();if(pros::millis()>=cancelTime)motion::cancel();};
 assert(motion::follow(path)==motion::Result::Cancelled);assert(chassis.leftCommand==0&&chassis.rightCommand==0);pass("cancel stops both motors");
 setup({0,0,0});pros::tick=[]{};assert(motion::follow(path)==motion::Result::LocalizationUnavailable);assert(chassis.leftCommand==0&&chassis.rightCommand==0);pass("stale EKF stops follower");
 setup({0,0,0});pros::tick=[]{ekf::update();};assert(motion::follow(path,{},500)==motion::Result::Timeout);assert(chassis.leftCommand==0&&chassis.rightCommand==0);pass("stalled route times out and stops motors");
 setup({0,0,0});pros::competition::connected=true;pros::competition::disabled=true;assert(motion::follow(path)==motion::Result::Cancelled);assert(chassis.leftCommand==0&&chassis.rightCommand==0);pass("competition disable cancels motion");
 setup({10,20,30});ekf::setEnabled(false);chassis.x=10;chassis.y=20;chassis.heading=30;
 assert(auton_trace::begin("Fallback"));auton_trace::service();auton_trace::end();auton_trace::service();
 {std::ifstream file(auton_trace::status().file);std::string text((std::istreambuf_iterator<char>(file)),{});assert(text.find(",odom")!=std::string::npos);}
 pros::usd::installed=false;assert(auton_trace::begin("No Card"));auton_trace::service();auton_trace::end();auton_trace::service();assert(!auton_trace::status().sdAvailable);
 pros::usd::installed=true;assert(auton_trace::begin("Queue Overflow"));
 const auto beforeDrops=auton_trace::status().dropped;
 for(std::size_t i=0;i<auton_trace::QUEUE_CAPACITY+10;++i){pros::clockMs+=20;auton_trace::capture();}
 assert(auton_trace::status().dropped>beforeDrops);
 auton_trace::end();for(int i=0;i<10;++i)auton_trace::service();auton_trace::closeFile();
 pass("logger supports EZ odometry fallback, missing SD, and bounded overflow");
 std::filesystem::remove_all(traceDirectory);

}
