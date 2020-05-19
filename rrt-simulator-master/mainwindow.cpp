#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    renderArea = ui->renderArea;
    rrt = renderArea->rrt;
    simulated = false;
}

/**
 * @brief Start the simulator.
 */
void MainWindow::on_startButton_clicked()
{
    if (simulated) {
        ui->statusBox->setText(tr("Please reset!"));
        renderArea->update();
        return;
    }
    simulated = true;
    // get step size and max iterations from GUI.
    rrt->setMaxIterations(ui->maxIterations->text().toInt());
    rrt->setStepSize(ui->stepSize->text().toInt());

    assert(rrt->step_size > 0);
    assert(rrt->max_iter > 0);

    // RRT Algorithm main loop
    int rrtType = 2;
    if(rrtType==1){
        bool success = rrt->plan();
        if(success){ui->statusBox->setText(tr("Reached Destination!"));}
        else{ui->statusBox->setText(tr("Exceeded max iterations!"));}
        rrt->nodes = rrt->tempNodes;
        rrt->findPath(success);
        renderArea->update();
    }else if(rrtType==2){
        rrt->planConnect();
        renderArea->update();
    }else if(rrtType==3){
        // TODO
        ;
    }else{
        cout << "Error: check your rrtType which is: " << rrtType << endl;
    }
}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked()
{
    simulated = false;
    ui->statusBox->setText(tr(""));
    rrt->obstacles->obstacles.clear();
    rrt->obstacles->obstacles.resize(0);
//    rrt->deleteNodes(rrt->root);
    rrt->nodes.clear();
    rrt->nodes.resize(0);
    rrt->tempNodes.clear();
    rrt->tempNodes.resize(0);
    rrt->path.clear();
    rrt->path.resize(0);
    rrt->initialize();
    renderArea->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
