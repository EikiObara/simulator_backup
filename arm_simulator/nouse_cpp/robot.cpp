#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodekits/SoCameraKit.h>
#include <Inventor/nodekits/SoLightKit.h>
#include <Inventor/nodekits/SoSceneKit.h>
#include <Inventor/nodekits/SoShapeKit.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/engines/SoCalculator.h>
#include <Inventor/engines/SoElapsedTime.h>
#include <Inventor/engines/SoTimeCounter.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/SbVec3f.h>
double a;
float a1, a2, a3;
float c1, c2, c3;
float h1, h2, h3;
float r1, r2, r3, r4;
float l1, l2, l3, l4;
float z1, z2, z3, z4, z5, z6, z7;

static void                            ////////////////////首を傾げる
headrotatingSensorCallback1(void *h_data1, SoSensor *)
{
	SoTransform *hTransform1 = (SoTransform *)h_data1;
	hTransform1->center.setValue(0.0, 30.0, 0.0);
	hTransform1->rotation.setValue(SbVec3f(0., 0., 1.), h1);
}

static void                            ////////////////////うなずく
headrotatingSensorCallback2(void *h_data2, SoSensor *)
{
	SoTransform *hTransform2 = (SoTransform *)h_data2;
	hTransform2->center.setValue(0.0, 17.5, 0.0);
	hTransform2->rotation.setValue(SbVec3f(1., 0., 0.), h2);
}

static void                            ////////////////////首を振る
headrotatingSensorCallback3(void *h_data3, SoSensor *)
{
	SoTransform *hTransform3 = (SoTransform *)h_data3;
	hTransform3->center.setValue(0.0, 10.5, 0.0);
	hTransform3->rotation.setValue(SbVec3f(0., 1., 0.), h3);
}

static void                           /////////////////////右腕を回す
rarmrotatingSensorCallback1(void *r_data1, SoSensor *)
{
	SoTransform *rTransform1 = (SoTransform *)r_data1;
	rTransform1->center.setValue(0.0, 0.0, 0.0);
	rTransform1->rotation.setValue(SbVec3f(1., 0., 0.), r1);
}

static void                          ///////////////////////右腕を挙げる
rarmrotatingSensorCallback2(void *r_data2, SoSensor *)
{
	SoTransform *rTransform2 = (SoTransform *)r_data2;
	rTransform2->center.setValue(41.0, 0.0, 0.0);
	rTransform2->rotation.setValue(SbVec3f(0., 0., 1.), r2);
}


static void                          //////////////////////右手首を回す
rarmrotatingSensorCallback3(void *r_data3, SoSensor *)
{
	SoTransform *rTransform3 = (SoTransform *)r_data3;
	rTransform3->center.setValue(0.0, 0.0, 0.0);
	rTransform3->rotation.setValue(SbVec3f(1., 0., 0.), r3);
}


static void                          //////////////////////右肘を曲げる
rarmrotatingSensorCallback4(void *r_data4, SoSensor *)
{
	SoTransform *rTransform4 = (SoTransform *)r_data4;
	rTransform4->center.setValue(88.0, 0.0, 0.0);
	rTransform4->rotation.setValue(SbVec3f(0., 0., 1.), r4);
}

static void                           /////////////////////左腕を回す
larmrotatingSensorCallback1(void *l_data1, SoSensor *)
{
	SoTransform *lTransform1 = (SoTransform *)l_data1;
	lTransform1->center.setValue(0.0, 0.0, 0.0);
	lTransform1->rotation.setValue(SbVec3f(1., 0., 0.), l1);
}

static void                          ///////////////////////左腕を挙げる
larmrotatingSensorCallback2(void *l_data2, SoSensor *)
{
	SoTransform *lTransform2 = (SoTransform *)l_data2;
	lTransform2->center.setValue(-41.0, 0.0, 0.0);
	lTransform2->rotation.setValue(SbVec3f(0., 0., 1.), l2);
}

static void                          //////////////////////左手首を回す
larmrotatingSensorCallback3(void *l_data3, SoSensor *)
{
	SoTransform *lTransform3 = (SoTransform *)l_data3;
	lTransform3->center.setValue(0.0, 0.0, 0.0);
	lTransform3->rotation.setValue(SbVec3f(1., 0., 0.), l3);
}

static void                          //////////////////////左肘を曲げる
larmrotatingSensorCallback4(void *l_data4, SoSensor *)
{
	SoTransform *lTransform4 = (SoTransform *)l_data4;
	lTransform4->center.setValue(-88.0, 0.0, 0.0);
	lTransform4->rotation.setValue(SbVec3f(0., 0., 1.), l4);
}

static void                            ////////////////////肩を動かす
boxtranslationSensorCallback1(void *b_data1, SoSensor *)
{
	SoTransform *bTransform1 = (SoTransform *)b_data1;
	bTransform1->translation.setValue(0, z1, 0);
	z1 = sin(z2);
	z2 += 0.03;
}

static void                            ////////////////////胸を上下させる
boxtranslationSensorCallback2(void *b_data2, SoSensor *)
{
	SoTransform *bTransform2 = (SoTransform *)b_data2;
	bTransform2->scaleFactor.setValue(z5, z5, 1);
	z5 = 1.006 + 0.006*z3;
	z3 = sin(z4);
	z4 += 0.03;
	// printf("%f\n",h4);
}

static void                          //////////////////////首の回転
boxrotatingSensorCallback3(void *b_data4, SoSensor *)
{
	SoTransform *bTransform3 = (SoTransform *)b_data4;
	bTransform3->rotation.setValue(SbVec3f(0, 1, 0), z6);
	z6 = 0.05*sin(z7);
	z7 += 0.03;
}

void
myKeyPressCB(void *userData, SoEventCallback *eventCB)
{

	const SoEvent *event = eventCB->getEvent();
	//頭を動かすkey
	////////////////////////////////////////////////////
	if (SO_KEY_PRESS_EVENT(event, Z)) {
		h1 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, X)) {
		h2 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, C)) {
		h3 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, V)) {
		h1 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, B)) {
		h2 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, N)) {
		h3 -= M_PI / 90;
	}
	///////////////////////////////////////////////////////
	//右腕を動かす
	//////////////////////////////////////////////////////
	if (SO_KEY_PRESS_EVENT(event, A)) {
		r1 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, S)) {
		r2 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, D)) {
		r3 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, F)) {
		r4 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, G)) {
		r1 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, H)) {
		r2 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, J)) {
		r3 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, K)) {
		r4 -= M_PI / 90;
	}
	///////////////////////////////////////////////////////
	//左腕を動かす
	//////////////////////////////////////////////////////
	if (SO_KEY_PRESS_EVENT(event, Q)) {
		l1 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, W)) {
		l2 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, E)) {
		l3 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, R)) {
		l4 += M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, T)) {
		l1 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, Y)) {
		l2 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, U)) {
		l3 -= M_PI / 90;
	}
	if (SO_KEY_PRESS_EVENT(event, I)) {
		l4 -= M_PI / 90;
	}
	///////////////////////////////////////////////////////

	eventCB->setHandled();
}


int
main(int, char **argv)
{
	HWND myWindow = SoWin::init(argv[0]);//radius
	if (myWindow == NULL) exit(1);

	SoSeparator *root = new SoSeparator;
	SoSeparator *MR1 = new SoSeparator;
	SoSeparator *MR2 = new SoSeparator;
	SoSeparator *MR3 = new SoSeparator;
	SoSeparator *MR4 = new SoSeparator;
	SoSeparator *MR5 = new SoSeparator;

	SoSeparator *ML1 = new SoSeparator;
	SoSeparator *ML2 = new SoSeparator;
	SoSeparator *ML3 = new SoSeparator;
	SoSeparator *ML4 = new SoSeparator;
	SoSeparator *ML5 = new SoSeparator;

	SoSeparator *MH1 = new SoSeparator;
	SoSeparator *MH2 = new SoSeparator;
	SoSeparator *MH3 = new SoSeparator;
	SoSeparator *MH4 = new SoSeparator;

	//謎の物体Ｘ
	SoCube *Box1 = new SoCube;
	Box1->width = 60;
	Box1->height = 25;
	Box1->depth = 26;

	SoSeparator *BOX1 = new SoSeparator;
	SoSeparator *BOX2 = new SoSeparator;

	SoMaterial *myMaterial = new SoMaterial;
	myMaterial->diffuseColor.setValue(1, 1, 1);








	//メタセコイアのデータを読み込みます

	SoInput myInput;
	if (!myInput.openFile("data/head_1.WRL"))
		return (1);
	SoSeparator *h1_fileContents = SoDB::readAll(&myInput);
	if (h1_fileContents == NULL) return (1);

	SoInput myInput2;
	if (!myInput.openFile("data/head_2.WRL"))
		return (1);
	SoSeparator *h2_fileContents = SoDB::readAll(&myInput);
	if (h2_fileContents == NULL) return (1);

	SoInput myInput3;
	if (!myInput.openFile("data/head_3.WRL"))

		return (1);
	SoSeparator *h3_fileContents = SoDB::readAll(&myInput);
	if (h3_fileContents == NULL) return (1);

	SoInput myInput4;
	if (!myInput.openFile("data/body.WRL"))
		return (1);
	SoSeparator *b_fileContents = SoDB::readAll(&myInput);
	if (b_fileContents == NULL) return (1);

	SoInput myInput5;
	if (!myInput.openFile("data/arm_right_1.WRL"))
		return (1);
	SoSeparator *ar1_fileContents = SoDB::readAll(&myInput);
	if (ar1_fileContents == NULL) return (1);

	SoInput myInput6;
	if (!myInput.openFile("data/arm_right_2.WRL"))
		return (1);
	SoSeparator *ar2_fileContents = SoDB::readAll(&myInput);
	if (ar2_fileContents == NULL) return (1);

	SoInput myInput7;
	if (!myInput.openFile("data/arm_right_3.WRL"))
		return (1);
	SoSeparator *ar3_fileContents = SoDB::readAll(&myInput);
	if (ar3_fileContents == NULL) return (1);

	SoInput myInput8;
	if (!myInput.openFile("data/arm_right_4.WRL"))
		return (1);
	SoSeparator *ar4_fileContents = SoDB::readAll(&myInput);
	if (ar4_fileContents == NULL) return (1);

	SoInput myInput9;
	if (!myInput.openFile("data/arm_left_1.WRL"))
		return (1);
	SoSeparator *al1_fileContents = SoDB::readAll(&myInput);
	if (al1_fileContents == NULL) return (1);

	SoInput myInput10;
	if (!myInput.openFile("data/arm_left_2.WRL"))
		return (1);
	SoSeparator *al2_fileContents = SoDB::readAll(&myInput);
	if (al2_fileContents == NULL) return (1);

	SoInput myInput11;
	if (!myInput.openFile("data/arm_left_3.WRL"))
		return (1);
	SoSeparator *al3_fileContents = SoDB::readAll(&myInput);
	if (al3_fileContents == NULL) return (1);

	SoInput myInput12;
	if (!myInput.openFile("data/arm_left_4.WRL"))
		return (1);
	SoSeparator *al4_fileContents = SoDB::readAll(&myInput);
	if (al4_fileContents == NULL) return (1);

	//パースカメラを定義する
	SoPerspectiveCamera *Camera = new SoPerspectiveCamera;


	//各オブジェクトのseparatorを作ります。
	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	SoSeparator *head1 = new SoSeparator;
	SoSeparator *head2 = new SoSeparator;
	SoSeparator *head3 = new SoSeparator;

	SoSeparator *body = new SoSeparator;

	SoSeparator *ri_arm1 = new SoSeparator;
	SoSeparator *ri_arm2 = new SoSeparator;
	SoSeparator *ri_arm3 = new SoSeparator;
	SoSeparator *ri_arm4 = new SoSeparator;

	SoSeparator *le_arm1 = new SoSeparator;
	SoSeparator *le_arm2 = new SoSeparator;
	SoSeparator *le_arm3 = new SoSeparator;
	SoSeparator *le_arm4 = new SoSeparator;

	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////


	//各オブジェクトの位置を決めます。
	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	SoTranslation *Transform1 = new SoTranslation;
	SoTranslation *Transform2 = new SoTranslation;
	SoTranslation *Transform3 = new SoTranslation;
	SoTranslation *Transform4 = new SoTranslation;
	SoTranslation *Transform5 = new SoTranslation;
	SoTranslation *Transform6 = new SoTranslation;
	SoTranslation *Transform7 = new SoTranslation;
	SoTranslation *Transform8 = new SoTranslation;
	SoTranslation *Transform9 = new SoTranslation;
	SoTranslation *Transform10 = new SoTranslation;
	SoTranslation *Transform11 = new SoTranslation;
	SoTranslation *Transform12 = new SoTranslation;

	SoTranslation *Transform = new SoTranslation;

	Transform1->translation.setValue(0.0, 7.5, 0.0);
	Transform2->translation.setValue(0.0, 17.5, 0.0);
	Transform3->translation.setValue(0.0, 31.5, 0.0);
	Transform4->translation.setValue(0.0, 0.0, 0.0);
	Transform5->translation.setValue(25.5, 0.0, 0.0);
	Transform6->translation.setValue(41.5, 0.0, 0.0);
	Transform7->translation.setValue(47.5, 0.0, 0.0);
	Transform8->translation.setValue(88.5, 0.0, 0.0);
	Transform9->translation.setValue(-25.5, 0.0, 0.0);
	Transform10->translation.setValue(-41.5, 0.0, 0.0);
	Transform11->translation.setValue(-47.5, 0.0, 0.0);

	Transform12->translation.setValue(-88.5, 0.0, 0.0);

	Transform->translation.setValue(0.0, -1.8, 0.0);


	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////

	//コールバック関数
	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	SoTransform *htransform1 = new SoTransform;
	SoTransform *htransform2 = new SoTransform;
	SoTransform *htransform3 = new SoTransform;

	SoTransform *rtransform1 = new SoTransform;
	SoTransform *rtransform2 = new SoTransform;
	SoTransform *rtransform3 = new SoTransform;
	SoTransform *rtransform4 = new SoTransform;

	SoTransform *ltransform1 = new SoTransform;
	SoTransform *ltransform2 = new SoTransform;
	SoTransform *ltransform3 = new SoTransform;
	SoTransform *ltransform4 = new SoTransform;

	SoTransform *bTransform1 = new SoTransform;
	SoTransform *bTransform2 = new SoTransform;
	SoTransform *bTransform3 = new SoTransform;

	SoEventCallback* EventCallback = new SoEventCallback;
	EventCallback->addEventCallback(SoKeyboardEvent::getClassTypeId(), myKeyPressCB, root);

	//head
	SoTimerSensor *headrotatingSensor1 =
		new SoTimerSensor(headrotatingSensorCallback1, htransform1);
	SoTimerSensor *headrotatingSensor2 =
		new SoTimerSensor(headrotatingSensorCallback2, htransform2);
	SoTimerSensor *headrotatingSensor3 =
		new SoTimerSensor(headrotatingSensorCallback3, htransform3);

	//right-arm   
	SoTimerSensor *rarmrotatingSensor1 =
		new SoTimerSensor(rarmrotatingSensorCallback1, rtransform1);
	SoTimerSensor *rarmrotatingSensor2 =
		new SoTimerSensor(rarmrotatingSensorCallback2, rtransform2);
	SoTimerSensor *rarmrotatingSensor3 =
		new SoTimerSensor(rarmrotatingSensorCallback3, rtransform3);
	SoTimerSensor *rarmrotatingSensor4 =
		new SoTimerSensor(rarmrotatingSensorCallback4, rtransform4);

	//left-arm   
	SoTimerSensor *larmrotatingSensor1 =
		new SoTimerSensor(larmrotatingSensorCallback1, ltransform1);
	SoTimerSensor *larmrotatingSensor2 =
		new SoTimerSensor(larmrotatingSensorCallback2, ltransform2);
	SoTimerSensor *larmrotatingSensor3 =
		new SoTimerSensor(larmrotatingSensorCallback3, ltransform3);
	SoTimerSensor *larmrotatingSensor4 =
		new SoTimerSensor(larmrotatingSensorCallback4, ltransform4);


	SoTimerSensor *boxtranslationSensor1 =
		new SoTimerSensor(boxtranslationSensorCallback1, bTransform1);
	SoTimerSensor *boxtranslationSensor2 =
		new SoTimerSensor(boxtranslationSensorCallback2, bTransform2);
	SoTimerSensor *boxtranslationSensor3 =
		new SoTimerSensor(boxrotatingSensorCallback3, bTransform3);



	headrotatingSensor1->setInterval(0.1); // scheduled once per second
	headrotatingSensor1->schedule();
	headrotatingSensor2->setInterval(0.1); // scheduled once per second
	headrotatingSensor2->schedule();
	headrotatingSensor3->setInterval(0.1); // scheduled once per second
	headrotatingSensor3->schedule();

	rarmrotatingSensor1->setInterval(0.1); // scheduled once per second
	rarmrotatingSensor1->schedule();
	rarmrotatingSensor2->setInterval(0.1); // scheduled once per second
	rarmrotatingSensor2->schedule();
	rarmrotatingSensor3->setInterval(0.1); // scheduled once per second
	rarmrotatingSensor3->schedule();
	rarmrotatingSensor4->setInterval(0.1); // scheduled once per second
	rarmrotatingSensor4->schedule();

	larmrotatingSensor1->setInterval(0.1); // scheduled once per second
	larmrotatingSensor1->schedule();
	larmrotatingSensor2->setInterval(0.1); // scheduled once per second
	larmrotatingSensor2->schedule();
	larmrotatingSensor3->setInterval(0.1); // scheduled once per second
	larmrotatingSensor3->schedule();
	larmrotatingSensor4->setInterval(0.1); // scheduled once per second
	larmrotatingSensor4->schedule();

	boxtranslationSensor1->setInterval(0.01); // scheduled once per second
	boxtranslationSensor1->schedule();
	boxtranslationSensor2->setInterval(0.01); // scheduled once per second
	boxtranslationSensor2->schedule();
	boxtranslationSensor3->setInterval(0.01); // scheduled once per second
	boxtranslationSensor3->schedule();




	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////

	//各オブジェクトの中身を設定します。
	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	head1->addChild(Transform1);
	head2->addChild(Transform2);
	head3->addChild(Transform3);
	body->addChild(Transform4);
	ri_arm1->addChild(Transform5);
	ri_arm2->addChild(Transform6);
	ri_arm3->addChild(Transform7);
	ri_arm4->addChild(Transform8);
	le_arm1->addChild(Transform9);
	le_arm2->addChild(Transform10);
	le_arm3->addChild(Transform11);
	le_arm4->addChild(Transform12);


	head1->addChild(h1_fileContents);
	head2->addChild(h2_fileContents);
	head3->addChild(h3_fileContents);
	body->addChild(b_fileContents);
	ri_arm1->addChild(ar1_fileContents);
	ri_arm2->addChild(ar2_fileContents);
	ri_arm3->addChild(ar3_fileContents);
	ri_arm4->addChild(ar4_fileContents);
	le_arm1->addChild(al1_fileContents);
	le_arm2->addChild(al2_fileContents);
	le_arm3->addChild(al3_fileContents);
	le_arm4->addChild(al4_fileContents);


	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////

	//右腕の動き  
	///////////////////////////////////////////////////////////
	MR1->addChild(rtransform4);
	MR1->addChild(ri_arm4);

	MR2->addChild(rtransform3);
	MR2->addChild(ri_arm3);
	MR2->addChild(MR1);

	MR3->addChild(rtransform2);
	MR3->addChild(ri_arm2);
	MR3->addChild(MR2);

	MR4->addChild(rtransform1);
	MR4->addChild(ri_arm1);
	MR4->addChild(MR3);

	MR5->addChild(bTransform1);
	MR5->addChild(MR4);

	//左腕の動き  
	///////////////////////////////////////////////////////////
	ML1->addChild(ltransform4);
	ML1->addChild(le_arm4);

	ML2->addChild(ltransform3);
	ML2->addChild(le_arm3);
	ML2->addChild(ML1);

	ML3->addChild(ltransform2);
	ML3->addChild(le_arm2);
	ML3->addChild(ML2);

	ML4->addChild(ltransform1);
	ML4->addChild(le_arm1);
	ML4->addChild(ML3);

	ML5->addChild(bTransform1);
	ML5->addChild(ML4);

	//頭の動き
	//////////////////////////////////////////////   
	MH1->addChild(htransform1);
	MH1->addChild(head3);

	MH2->addChild(htransform2);
	MH2->addChild(head2);
	MH2->addChild(MH1);

	MH3->addChild(htransform3);
	MH3->addChild(head1);
	MH3->addChild(MH2);

	MH4->addChild(bTransform3);
	MH4->addChild(MH3);
	////////////////////////////////////////////////

	BOX1->addChild(Transform);
	BOX1->addChild(myMaterial);
	BOX1->addChild(Box1);

	BOX2->addChild(bTransform2);
	BOX2->addChild(BOX1);

	root->ref();
	/*root->addChild(Camera);*/

	root->addChild(EventCallback);

	root->addChild(body);
	root->addChild(MH4);
	root->addChild(MR5);
	root->addChild(ML5);
	root->addChild(BOX2);


	SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
	myViewer->setSceneGraph(root);
	myViewer->setTitle("arm-joint");
	myViewer->show();

	SoWin::show(myWindow);
	SoWin::mainLoop();
}