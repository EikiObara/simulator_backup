// 2017/04/28
// created by eiki obara

#include <Inventor\Win\SoWin.h>
#include <Inventor\So.h>
#include <vector>
#include <time.h>

inline void InitRand() {
	srand((unsigned int)time(NULL));
}

//座標点確認用サクランボ描画
void GoalCherry(std::vector<double> position, SoSeparator *GoalSep) {
	///////////////////////////////////////

	SoInput GoalIpt;
	if (!GoalIpt.openFile("cherryfruit.wrl")) exit(1);
	SoSeparator *GoalObj1 = SoDB::readAll(&GoalIpt);
	if (GoalObj1 == NULL) exit(1);

	/*
	SoSphere *GoalObj = new SoSphere();
	SoTransform *GoalTransform = new SoTransform;
	GoalTransform->scaleFactor.setValue(0.002, 0.002, 0.002);
	*/
	//color
	SoMaterial *GoalColor = new SoMaterial;
	GoalColor->diffuseColor.setValue(1, 0, 0);

	//位置合わせ
	SoTranslation *GoalTranslation1 = new SoTranslation;
	//GoalTranslation1->translation.setValue(cherry[0], cherry[1] - 0.03, cherry[2]);
	GoalTranslation1->translation.setValue(position[0], position[1] - 0.03, position[2]);

	SoTransform *GoalInitialrot = new SoTransform;	//Y

	GoalSep->addChild(GoalColor);
	GoalSep->addChild(GoalTranslation1);
	//GoalSep->addChild(GoalTransform);
	GoalSep->addChild(GoalObj1);
}

void PointObj(std::vector<double> position, SoSeparator *redPoint) {
	InitRand();

	SoSphere *redP = new SoSphere;

	redP->radius = 0.01;

	SoMaterial *myMaterial = new SoMaterial;
	myMaterial->diffuseColor.setValue(1, 0, 0);
	//myMaterial->diffuseColor.setValue((rand() % 101) * 0.01, (rand() % 101) * 0.01, (rand() % 101) * 0.01);

	SoTranslation *trans = new SoTranslation;

	trans->translation.setValue(position[0], position[1], position[2]);

	redPoint->addChild(myMaterial);
	redPoint->addChild(trans);
	redPoint->addChild(redP);
}

void PointObj(SoSeparator *redPoint) {
	InitRand();

	SoSphere *pointObj = new SoSphere;

	pointObj->radius = 0.01;

	SoMaterial *red= new SoMaterial;
	red->diffuseColor.setValue((rand() % 101) * 0.01, (rand() % 101) * 0.01, (rand() % 101) * 0.01);

	redPoint->addChild(red);
	redPoint->addChild(pointObj);
}

void AlmiFlame(std::vector<double> position,SoSeparator *armFlame) {
	///////////////////////////////////////

	SoInput armFlameIpt;
	if (!armFlameIpt.openFile("hfsh8-8080-1000_vertical.wrl")) exit(1);
	SoSeparator *armFlameObj = SoDB::readAll(&armFlameIpt);
	if (armFlameObj == NULL) exit(1);

	// no.1 flame position (0.11, -0.916, -0.210)
	// no.2 flame position (-0.11, -0.916, -0.210)

	/*
	SoSphere *GoalObj = new SoSphere();
	SoTransform *GoalTransform = new SoTransform;
	GoalTransform->scaleFactor.setValue(0.002, 0.002, 0.002);
	*/
	//color
	SoMaterial *flameColor = new SoMaterial;
	flameColor->diffuseColor.setValue(0.7, 0.7, 0.7);

	//位置合わせ
	SoTranslation *positionConfig = new SoTranslation;
	positionConfig->translation.setValue(position[0], position[1], position[2]);

	armFlame->addChild(flameColor);
	armFlame->addChild(positionConfig);
	//GoalSep->addChild(GoalTransform);
	armFlame->addChild(armFlameObj);
}

//座標系
void CoordinateSystem(SoSeparator *coordinateSystem) {
	SoFont *myFont = new SoFont;
	myFont->name.setValue("Tmes-Roman");
	myFont->size.setValue(24.0);
	coordinateSystem->addChild(myFont);

	SoSeparator *X_text = new SoSeparator;
	SoSeparator *Y_text = new SoSeparator;
	SoSeparator *Z_text = new SoSeparator;

	SoText2 *Xplus = new SoText2;	Xplus->string = "X";
	SoText2 *Yplus = new SoText2;	Yplus->string = "Y";
	SoText2 *Zplus = new SoText2;	Zplus->string = "Z";

	SoTranslation *X_Position = new SoTranslation;
	SoTranslation *Y_Position = new SoTranslation;
	SoTranslation *Z_Position = new SoTranslation;

	X_Position->translation.setValue(0.15, 0, 0);
	Y_Position->translation.setValue(0, 0.15, 0);
	Z_Position->translation.setValue(0, 0, 0.15);

	SoMaterial *blue = new SoMaterial;
	SoMaterial *red = new SoMaterial;
	SoMaterial *green = new SoMaterial;

	red->diffuseColor.setValue(1, 0.1, 0.1);
	green->diffuseColor.setValue(0.1, 1, 0.1);
	blue->diffuseColor.setValue(0.1, 0.1, 1);

	X_text->addChild(X_Position);	X_text->addChild(blue);	X_text->addChild(Xplus);
	Y_text->addChild(Y_Position);	Y_text->addChild(red);	Y_text->addChild(Yplus);
	Z_text->addChild(Z_Position);	Z_text->addChild(green); Z_text->addChild(Zplus);

	coordinateSystem->addChild(X_text);
	coordinateSystem->addChild(Y_text);
	coordinateSystem->addChild(Z_text);
}

void ConeObj(SoSeparator *cone) {
	SoCone *c = new SoCone;

	c->bottomRadius = 0.01;
	c->height = 0.1;

	cone->addChild(c);
}