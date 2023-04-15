/**
 * @file /include/class1_ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for class1_ros_qt_demo.
 *
 * @date November 2010
 **/
#ifndef ui_main_MAIN_WINDOW_H
#define ui_main_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ui_main
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
		void on_btnexit_clicked();
		void on_btntaskmark_clicked();
		void on_btngo_clicked();
		void on_btnreturn_clicked();
		void on_btnposeset_clicked();
		void on_btnemer_clicked();

	private:
		Ui::MainWindowDesign ui;
		QNode qnode;
	};

} // namespace ui_main

#endif // ui_main_MAIN_WINDOW_H
