#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// #include <QMainWindow>
#include <QtWidgets/qmainwindow.h>
#include "renderarea.h"
#include "rrt.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private:
    Ui::MainWindow *ui;
    RenderArea *renderArea;
    RRT *rrt;
    RRTMult *rrtMult;
    bool simulated;
private slots:
    void on_startButton_clicked();
    void on_resetButton_clicked();

    //Following code are added to test algorithm efficientcy
    void generateObstacles();
    void recordTime(double time);
    void recordLength(int length);
};

#endif // MAINWINDOW_H
