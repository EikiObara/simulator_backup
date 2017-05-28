SoSeparator *root = new SoSeparator;
root->ref();

////���������
SoSeparator *scene = new SoSeparator;
SoPerspectiveCamera *myCamera = new SoPerspectiveCamera;
SoPointLight *myLight = new SoPointLight;

myLight->location.setValue(5, 10, 15);
scene->addChild(myCamera);
scene->addChild(myLight);

//�Z�p���[�^�[�����
SoSeparator *SPHERE1 = new SoSeparator;
SoSeparator *SPHERE2 = new SoSeparator;
SoSeparator *SPHERE3 = new SoSeparator;

////�I�u�W�F�N�g�����
SoSphere *Sphere = new SoSphere;
Sphere->radius = 10;
SoSphere *Sphere2 = new SoSphere;
Sphere2->radius = 3;
SoSphere *Sphere3 = new SoSphere;
Sphere3->radius = 1;

//�F������
SoMaterial *pink = new SoMaterial;
/*
pink->diffuseColor.setValue(.8, .2, .4);//���U������A�L�߂�
pink->specularColor.setValue(1, 0, 0);	//���̂悤�ɔ��˂��邱�Ƃ��ł���
pink->ambientColor.setValue(1, 0, 0);	//��͂����A��芪��
pink->emissiveColor.setValue(1, 0, 0);	//���ː���
pink->shininess = 1.;
*/
root->addChild(pink);

////�ʒu�����߂�
SoTranslation *Transform1 = new SoTranslation;
Transform1->translation.setValue(-10, 0, -100);
SoTranslation *Transform2 = new SoTranslation;
Transform2->translation.setValue(10, 0, 10);


////�e�I�u�W�F�N�g�̒��g��ݒ肵�܂��B
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