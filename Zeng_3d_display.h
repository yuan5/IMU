#ifndef ZENG_3D_DISPLAY_H
#define ZENG_3D_DISPLAY_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <QOpenGLTexture>

class Zeng_3d_display:public QOpenGLWidget,protected QOpenGLFunctions
{
	Q_OBJECT
public:
	Zeng_3d_display(QWidget *parent);

protected:
	void paintGL() override;
	   //初始化GL
	void initializeGL() override;
	   //窗口尺寸变化
    void resizeGL(int w,int h) override;
private:
    void initialTexture(void);
    QOpenGLTexture* HD_texture = nullptr;
    QImage* texture_image = nullptr;
};

#endif // ZENG_3D_DISPLAY_H
