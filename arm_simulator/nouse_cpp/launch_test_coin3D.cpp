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
	//Open Inventorを初期化し、
	//Open Inventor Xライブラリを使用してウィンドウを作成する
	HWND myWindow = SoWin::init(argv[0]);
	if (myWindow == NULL)exit(1);

	//シーンを作成する。
	SoSeparator *root1 = new SoSeparator;
	SoSeparator *root2 = new SoSeparator;
	root1->ref();
	root2->ref();
	
	//カメラを作り、シーンに追加する。
	SoPerspectiveCamera *myCamera = new SoPerspectiveCamera;
	root1->addChild(myCamera);
	root2->addChild(myCamera);
	//シーンに高原を追加する。位置はデフォルト。
	root1->addChild(new SoDirectionalLight);
	root2->addChild(new SoDirectionalLight);

	//材質を作り、色を赤にし、シーンに追加する。
	SoMaterial *myMaterial1 = new SoMaterial;
	SoMaterial *myMaterial2 = new SoMaterial;
	myMaterial1->diffuseColor.setValue(1, 0, 0);
	myMaterial2->diffuseColor.setValue(0, 1, 0);
	root1->addChild(myMaterial1);
	root2->addChild(myMaterial2);

	//シーンに円錐を追加する
	root1->addChild(new SoCone);
	root2->addChild(new SoCone);

	//Winユーティリティライブラリを使用し、描画エリアをつくる。
	//描画エリアはメインウィンドウに現れる。
	SoWinRenderArea *myRenderArea = new SoWinRenderArea(myWindow);

	//シーン全体が見えるようにカメラを設定する。
	myCamera->viewAll(root1, myRenderArea->getViewportRegion());
	myCamera->viewAll(root2, myRenderArea->getViewportRegion());

	//myRenderAreaにシーンを移動し、タイトルを変更する。
	myRenderArea->setSceneGraph(root2);

	myRenderArea->setSceneGraph(root1);
	myRenderArea->setTitle("Hello Cone");
	myRenderArea->show();

	SoWin::show(myWindow);
	SoWin::mainLoop();

}