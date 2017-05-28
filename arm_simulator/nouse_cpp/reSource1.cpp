SoSeparator *root = new SoSeparator;
root->ref();

////光源を作る
SoSeparator *scene = new SoSeparator;
SoPerspectiveCamera *myCamera = new SoPerspectiveCamera;
SoPointLight *myLight = new SoPointLight;

myLight->location.setValue(5, 10, 15);
scene->addChild(myCamera);
scene->addChild(myLight);

//セパレーターを作る
SoSeparator *SPHERE1 = new SoSeparator;
SoSeparator *SPHERE2 = new SoSeparator;
SoSeparator *SPHERE3 = new SoSeparator;

////オブジェクトを作る
SoSphere *Sphere = new SoSphere;
Sphere->radius = 10;
SoSphere *Sphere2 = new SoSphere;
Sphere2->radius = 3;
SoSphere *Sphere3 = new SoSphere;
Sphere3->radius = 1;

//色をつける
SoMaterial *pink = new SoMaterial;
/*
pink->diffuseColor.setValue(.8, .2, .4);//放散させる、広める
pink->specularColor.setValue(1, 0, 0);	//鏡のように反射することができる
pink->ambientColor.setValue(1, 0, 0);	//包囲した、取り巻く
pink->emissiveColor.setValue(1, 0, 0);	//放射性の
pink->shininess = 1.;
*/
root->addChild(pink);

////位置を決める
SoTranslation *Transform1 = new SoTranslation;
Transform1->translation.setValue(-10, 0, -100);
SoTranslation *Transform2 = new SoTranslation;
Transform2->translation.setValue(10, 0, 10);


////各オブジェクトの中身を設定します。
SPHERE1->addChild(Transform1);
SPHERE1->addChild(Sphere);
SPHERE2->addChild(Transform2);
SPHERE2->addChild(Sphere);
SPHERE3->addChild(Sphere2);


root->addChild(SPHERE1);
root->addChild(SPHERE2);
root->addChild(SPHERE3);

SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
myViewer->setSceneGraph(root);
myViewer->setTitle("cherry");
myViewer->show();

SoWin::show(myWindow);
SoWin::mainLoop();