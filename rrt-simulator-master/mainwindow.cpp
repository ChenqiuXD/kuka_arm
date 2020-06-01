#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    renderArea = ui->renderArea;
    rrt = renderArea->rrt;
    rrtMult = renderArea->rrtMult;
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
    if(renderArea->planType==0){
        rrt->setMaxIterations(ui->maxIterations->text().toInt());
        rrt->setStepSize(ui->stepSize->text().toInt());
    }else if(renderArea->planType==1){
        rrtMult->setMaxIterations(ui->maxIterations->text().toInt());
        rrtMult->setStepSize(ui->stepSize->text().toInt());
    }

    // RRT Algorithm main loop
    // 1->original RRT, 2->seperate and link RRT, 3->rrtMult
    int rrtType = 5;
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
        rrtMult->plan();
        renderArea->update();
    }else if(rrtType == 4){
        bool success = rrt->planBi();
        rrt->nodes = rrt->tempNodes;
        rrt->nodes.insert(rrt->nodes.end(), rrt->backTree.begin(), rrt->backTree.end());
        rrt->findPathBi(success);
        renderArea->update();
    }else if(rrtType == 5){
        bool success = rrt->planBiConnect();
        rrt->nodes = rrt->tempNodes;
        rrt->nodes.insert(rrt->nodes.end(), rrt->backTree.begin(), rrt->backTree.end());
        rrt->findPathBiConnect(success);
        renderArea->update();
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
    if(renderArea->planType==0){
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
    }else if(renderArea->planType==1){
        rrtMult->obstacles->obstacles.clear();
        rrtMult->obstacles->obstacles.resize(0);
        rrtMult->nodes.clear();
        rrtMult->nodes.resize(0);
        rrtMult->tempNodes.clear();
        rrtMult->tempNodes.resize(0);
        rrtMult->path.clear();
        rrtMult->path.resize(0);
        rrtMult->initialize();
    }

    renderArea->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
