#include "dual_foc.h"
#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	dual_foc w;
	w.setWindowTitle("HD IMU");
	w.show();

	return a.exec();
}
