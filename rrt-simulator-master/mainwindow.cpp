#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <chrono>

using namespace std::chrono;

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

    // Following code are used for testing algorithm efficiency
    generateObstacles();
    auto start = high_resolution_clock::now();
    auto stop = high_resolution_clock::now();

    // RRT Algorithm main loop
    // 1->original RRT, 2->seperate and link RRT, 3->rrtMult
    int rrtType = 1;
    bool success;
    if(rrtType==1){
        success = rrt->plan();
        // if(success){ui->statusBox->setText(tr("Reached Destination!"));}
        // else{ui->statusBox->setText(tr("Exceeded max iterations!"));}
        rrt->findPath(success);
        stop = high_resolution_clock::now();
        rrt->nodes = rrt->tempNodes;
        renderArea->update();
    }else if(rrtType==2){
        success = rrt->planConnect();
        stop = high_resolution_clock::now();
        renderArea->update();
    }else if(rrtType==3){
        success = rrtMult->plan();
        stop = high_resolution_clock::now();
        renderArea->update();
    }else if(rrtType == 4){
        success = rrt->planBi();
        rrt->findPathBi(success);
        stop = high_resolution_clock::now();
        rrt->nodes = rrt->tempNodes;
        rrt->nodes.insert(rrt->nodes.end(), rrt->backTree.begin(), rrt->backTree.end());
        renderArea->update();
    }else if(rrtType == 5){
        success = rrt->planBiConnect();
        rrt->findPathBiConnect(success);
        stop = high_resolution_clock::now();
        rrt->nodes = rrt->tempNodes;
        rrt->nodes.insert(rrt->nodes.end(), rrt->backTree.begin(), rrt->backTree.end());
        renderArea->update();
    }else{
        cout << "Error: check your rrtType which is: " << rrtType << endl;
    }
    auto duration = duration_cast<microseconds>(stop-start);
    cout << (success?"Successfully":"Failed")<< " to plan with elapsed time is: " << duration.count() << " microseconds" << endl;
}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked()
{
    simulated = false;
    ui->statusBox->setText(tr(""));
    if(renderArea->planType==0){
        // rrt->obstacles->obstacles.clear();
        // rrt->obstacles->obstacles.resize(0);
    //    rrt->deleteNodes(rrt->root);
        rrt->nodes.clear();
        rrt->nodes.resize(0);
        rrt->tempNodes.clear();
        rrt->tempNodes.resize(0);
        rrt->path.clear();
        rrt->path.resize(0);
        rrt->initialize();
    }else if(renderArea->planType==1){
        // rrtMult->obstacles->obstacles.clear();
        // rrtMult->obstacles->obstacles.resize(0);
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

void MainWindow::generateObstacles()
{
    vector<Vector2f> pointPos;
    pointPos.push_back(Vector2f(120,0));
    pointPos.push_back(Vector2f(140,100));

    pointPos.push_back(Vector2f(0,100));
    pointPos.push_back(Vector2f(80,120));

    pointPos.push_back(Vector2f(200,0));
    pointPos.push_back(Vector2f(220,280));

    // pointPos.push_back(Vector2f(130,160));
    // pointPos.push_back(Vector2f(150,400));

    pointPos.push_back(Vector2f(250,255));
    pointPos.push_back(Vector2f(400,275));

    pointPos.push_back(Vector2f(250,320));
    pointPos.push_back(Vector2f(270,400));

    for(size_t i=0;i<pointPos.size();i+=2){
        this->renderArea->rrt->obstacles->addObstacle(pointPos[i],pointPos[i+1]);
        this->renderArea->rrtMult->obstacles->addObstacle(pointPos[i],pointPos[i+1]);
    }
}

void MainWindow::recordTime(double time)
{
    ;
}

void MainWindow::recordLength(int length)
{
    ;
}

MainWindow::~MainWindow()
{
    delete ui;
}
