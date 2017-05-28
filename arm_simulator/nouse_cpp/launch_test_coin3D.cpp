#include <Inventor\Win\SoWin.h>
#include <Inventor\Win\viewers\SoWinExaminerViewer.h>
#include <Inventor\SoDB.h>
#include <Inventor\nodes\SoMaterial.h>
#include <Inventor\nodes\SoRotationXYZ.h>
#include <Inventor\nodes\SoScale.h>
#include <Inventor\nodes\SoCone.h>
#include <Inventor\nodes\SoCube.h>
#include <Inventor\nodes\SoSeparator.h>
#include <Inventor\nodes\SoTranslation.h>
#include <Inventor\nodes\SoPerspectiveCamera.h>
#include <Inventor\SoInput.h>
#include <Inventor\nodes\SoDirectionalLight.h>
#include <stdlib.h>

int main(int argc,char **argv)
{
	//Open Inventor�����������A
	//Open Inventor X���C�u�������g�p���ăE�B���h�E���쐬����
	HWND myWindow = SoWin::init(argv[0]);
	if (myWindow == NULL)exit(1);

	//�V�[�����쐬����B
	SoSeparator *root1 = new SoSeparator;
	SoSeparator *root2 = new SoSeparator;
	root1->ref();
	root2->ref();
	
	//�J���������A�V�[���ɒǉ�����B
	SoPerspectiveCamera *myCamera = new SoPerspectiveCamera;
	root1->addChild(myCamera);
	root2->addChild(myCamera);
	//�V�[���ɍ�����ǉ�����B�ʒu�̓f�t�H���g�B
	root1->addChild(new SoDirectionalLight);
	root2->addChild(new SoDirectionalLight);

	//�ގ������A�F��Ԃɂ��A�V�[���ɒǉ�����B
	SoMaterial *myMaterial1 = new SoMaterial;
	SoMaterial *myMaterial2 = new SoMaterial;
	myMaterial1->diffuseColor.setValue(1, 0, 0);
	myMaterial2->diffuseColor.setValue(0, 1, 0);
	root1->addChild(myMaterial1);
	root2->addChild(myMaterial2);

	//�V�[���ɉ~����ǉ�����
	root1->addChild(new SoCone);
	root2->addChild(new SoCone);

	//Win���[�e�B���e�B���C�u�������g�p���A�`��G���A������B
	//�`��G���A�̓��C���E�B���h�E�Ɍ����B
	SoWinRenderArea *myRenderArea = new SoWinRenderArea(myWindow);

	//�V�[���S�̂�������悤�ɃJ������ݒ肷��B
	myCamera->viewAll(root1, myRenderArea->getViewportRegion());
	myCamera->viewAll(root2, myRenderArea->getViewportRegion());

	//myRenderArea�ɃV�[�����ړ����A�^�C�g����ύX����B
	myRenderArea->setSceneGraph(root2);

	myRenderArea->setSceneGraph(root1);
	myRenderArea->setTitle("Hello Cone");
	myRenderArea->show();

	SoWin::show(myWindow);
	SoWin::mainLoop();

}