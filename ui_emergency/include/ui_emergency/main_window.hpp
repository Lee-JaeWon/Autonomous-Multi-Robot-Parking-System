/**
 * @file /include/class1_ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for class1_ros_qt_demo.
 *
 * @date November 2010
 **/
#ifndef ui_emergency_MAIN_WINDOW_H
#define ui_emergency_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ui_emergency
{

	/*****************************************************************************
	** Interface [MainWindow]
	*****************************************************************************/
	/**
	 * @brief Qt central, all operations relating to the view part here.
	 */
	class MainWindow : public QMainWindow
	{
		Q_OBJECT

	public:
		MainWindow(int argc, char **argv, QWidget *parent = 0);
		~MainWindow();

	public Q_SLOTS:

		// void on_actionAbout_triggered();
		void on_pushButton_clicked();
		void on_btnexit_clicked();

	private:
		Ui::MainWindowDesign ui;
		QNode qnode;
	};

} // namespace ui_emergency

#endif // ui_emergency_MAIN_WINDOW_H
