#include <iostream>
#include <Inventor\So.h>
#include <Inventor\events\SoEvent.h>
#include <Inventor\Win\SoWin.h>
#include <Inventor\events\SoSpaceballButtonEvent.h>
#include <Inventor\Win\devices\SoWinSpaceball.h>
#include <Inventor\Win\viewers\SoWinExaminerViewer.h>
#include <Inventor\sensors\SoTimerSensor.h>

#define SPACEMOUSE_TRANSLATION_GAIN 0.05f
#define SPACEMOUSE_ROTATION_GAIN 0.05f

float SpacemouseInput[6];

static void SpaceMouseXaxisCB(void *b_data, SoSensor *) {
	SoTransform *XTransform = (SoTransform *)b_data;
	XTransform->rotation.setValue(SbVec3f(1, 0, 0), SpacemouseInput[3]);
}

static void SpaceMouseYaxisCB(void *b_data, SoSensor *) {
	SoTransform *YTransform = (SoTransform *)b_data;
	YTransform->rotation.setValue(SbVec3f(0, 1, 0), -SpacemouseInput[5]);
}

static void SpaceMouseZaxisCB(void *b_data, SoSensor *) {
	SoTransform *ZTransform = (SoTransform *)b_data;
	ZTransform->rotation.setValue(SbVec3f(0, 0, 1), SpacemouseInput[4]);
}


SoWinSpaceball* SpaceMouseSet(){
	float f;
	if (SoWinSpaceball::exists()) {
		printf("スペースマウスを検出できません\n");
		exit(1);
	}
	SoWinSpaceball* sb = new SoWinSpaceball;
	f = sb->getRotationScaleFactor();
	printf("Spacemouse::RotationScaleFactor\t= %f\n", f);
	f = sb->getTranslationScaleFactor();
	printf("Spacemouse::TranslationScaleFactor\t= %f\n", f);
	return sb;
}

void Coin3DSpaceMouseEventCB(void* data, SoEventCallback *eventCB){
	//printf("event callback started\n");
	const SoEvent* ev = eventCB->getEvent();
	if (ev->isOfType(SoMotion3Event::getClassTypeId())) {
		SoMotion3Event* tr = (SoMotion3Event*)ev;
		SpacemouseInput[0] += SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[0];
		SpacemouseInput[1] += SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[1];
		SpacemouseInput[2] += SPACEMOUSE_TRANSLATION_GAIN * tr->getTranslation().getValue()[2];
		SpacemouseInput[3] += SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[0];
		SpacemouseInput[4] += SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[1];
		SpacemouseInput[5] += SPACEMOUSE_ROTATION_GAIN * tr->getRotation().getValue()[2];
	}
}

int main(void) {
	printf("start this program\n");

	HWND MainWindow = SoWin::init("");
	if (MainWindow == NULL) exit(1);

	printf("Success main window creation\n");

	//root separator create
	SoSeparator *root = new SoSeparator;
	root->ref();

	//viewer configration
	SoWinExaminerViewer *MainViewer = new SoWinExaminerViewer(MainWindow);
	MainViewer->setSceneGraph(root);
	MainViewer->setTitle("It is testing!");
	MainViewer->setSize(SbVec2s(640, 480));
	MainViewer->setBackgroundColor(SbColor(0, 0, 0));
	printf("Success main viewer configuration\n");
	
	if (SoWinSpaceball::exists()) {		printf("space ball NO exists\n");	exit(1);	}

	//space mouse callback create

	SoTransform *SpaceMouseTransformX = new SoTransform;
	SoTransform *SpaceMouseTransformY = new SoTransform;
	SoTransform *SpaceMouseTransformZ = new SoTransform;

	SoEventCallback *SpaceMouseCB = new SoEventCallback;
	SpaceMouseCB->addEventCallback(SoMotion3Event::getClassTypeId(), Coin3DSpaceMouseEventCB, root);
	
	SoTimerSensor *SpaceMouseSensorX = new SoTimerSensor(SpaceMouseXaxisCB, SpaceMouseTransformX);
	SpaceMouseSensorX->setInterval(0.1);
	SpaceMouseSensorX->schedule();

	SoTimerSensor *SpaceMouseSensorY = new SoTimerSensor(SpaceMouseYaxisCB, SpaceMouseTransformY);
	SpaceMouseSensorY->setInterval(0.1);
	SpaceMouseSensorY->schedule();

	SoTimerSensor *SpaceMouseSensorZ = new SoTimerSensor(SpaceMouseZaxisCB, SpaceMouseTransformZ);
	SpaceMouseSensorZ->setInterval(0.1);
	SpaceMouseSensorZ->schedule();


	SoSeparator *obj1 = new SoSeparator;
	SoSeparator *obj2 = new SoSeparator;
	SoSeparator *obj3 = new SoSeparator;
	SoCube *box1 = new SoCube;
	box1->width = 1.0;
	box1->height = 0.1;
	box1->depth = 0.1;
	SoCube *box2 = new SoCube;
	box2->width = 0.1;
	box2->height = 1.0;
	box2->depth = 0.1;
	SoCube *box3 = new SoCube;
	box3->width = 0.1;
	box3->height = 0.1;
	box3->depth = 1.0;

	SoTranslation *trans1 = new SoTranslation;
	SoTranslation *trans2 = new SoTranslation;
	SoTranslation *trans3 = new SoTranslation;

	trans1->translation.setValue(-0.5, 0, 0);
	trans2->translation.setValue(0, 0, 0);
	trans3->translation.setValue(0.5, 0, 0);

	obj1->addChild(trans1);
	obj1->addChild(SpaceMouseTransformX);
	obj1->addChild(box1);

	obj2->addChild(trans2);
	obj2->addChild(SpaceMouseTransformY);
	obj2->addChild(box2);

	obj3->addChild(trans3);
	obj3->addChild(SpaceMouseTransformZ);
	obj3->addChild(box3);

	root->addChild(SpaceMouseCB);
	root->addChild(obj1);
	root->addChild(obj2);
	root->addChild(obj3);

	MainViewer->registerDevice(SpaceMouseSet());

	MainViewer->show();
	SoWin::show(MainWindow);
	SoWin::mainLoop();
}