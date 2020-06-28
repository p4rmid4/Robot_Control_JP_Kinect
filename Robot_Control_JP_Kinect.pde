/*
Approach to send URScript commands to a UR-Robot 
 URScript Reference: http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf
 */

import processing.net.*;
import java.nio.*;
import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;

PVector[] jointVecs = new PVector[4];
int currentUser = 0;

URobot robot;


void setup() {
  //fullScreen(P3D);
  size(1920, 1080, P3D);
robot = new URobot(this, "192.168.9.2"); 
  kinect = new KinectPV2(this);
  kinect.enableSkeletonColorMap(true);
  kinect.enableColorImg(true);
  kinect.enableSkeleton3DMap(true);
  kinect.init();
  for (int i = 0; i < jointVecs.length; i++) {
    jointVecs[i] = new PVector(width/2, height/2);
  }
}

void draw() {
  //background(0);
  image(kinect.getColorImage(), 0, 0,width,height); 
  
  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();
  ArrayList<KSkeleton> skeletonArrayZ = kinect.getSkeleton3d();    

  
  int currentUserIndex = 0;
  if(skeletonArray != null && skeletonArray.size() > 0) {
    currentUserIndex = currentUser % skeletonArray.size();
    fill(skeletonArray.get(currentUserIndex).getIndexColor());
    textSize(32);
    text("the one in charge!", 20, 40);
  }
  
  //individual JOINTS
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
      KSkeleton skeletonZ = (KSkeleton) skeletonArrayZ.get(i); 
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();
      KJoint[] jointsZ = skeletonZ.getJoints();  
      color col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);
      drawBody(joints);
      
      drawJoint(joints, KinectPV2.JointType_HandRight);
      //draw different color for each hand state
      drawHandState(joints[KinectPV2.JointType_HandRight]);
      drawHandState(joints[KinectPV2.JointType_HandLeft]);
      
      if(i == currentUserIndex && joints[KinectPV2.JointType_HandLeft].getState() == KinectPV2.HandState_Closed)  {
        float x = constrain(joints[KinectPV2.JointType_HandRight].getX(),400,1400);
        float y = constrain(joints[KinectPV2.JointType_HandRight].getY(),200,800);
        float z = jointsZ[KinectPV2.JointType_HandRight].getZ();
      
        PVector kinectpt = new PVector(x, y, z); 
       //println(x,y,z);
        jointVecs[0] = new PVector(map(kinectpt.x, 400,1400,  PI/4, -PI/4),
                                   map(kinectpt.y, 200,800, PI/4, -PI/4),
                                   map(kinectpt.z*1000, 900,3000, PI/4, -PI/4));
           
        float dist = PVector.dist(jointVecs[0], jointVecs[1]);
        JointPose c;
        c = new JointPose(jointVecs[0].x, -PI/2, jointVecs[0].y, jointVecs[0].z, PI/2, 0);
        
        if(dist > 0.001 && !robot.robotModeData.isProgramRunning)  {
             robot.movej(c, 0.1);
        }
        
        jointVecs[1] = jointVecs[0].copy();
      }
    }
  }
 
  robot.test();
  
  
}


void keyPressed() {
  if (key == 'h') {
    robot.moveHome();
  }
}
