#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <chrono>
#include <stdlib.h>
#include <time.h>

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
    fout.open("test.txt");
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
    this->isObsRandom = true;
    generateObstacles();
    auto start = high_resolution_clock::now();
    auto stop = high_resolution_clock::now();

    // RRT Algorithm main loop
    // 1->original RRT, 2->seperate and link RRT, 3->rrtMult
    int rrtType = 6;
    bool success;
    if(rrtType==1){
        success = rrt->plan();
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
    }else if(rrtType == 6){
        success = rrt->planMultConnect();
        rrt->findPathMultConnect(success);
        stop = high_resolution_clock::now();
        rrt->nodes = rrt->tempNodes;
        rrt->nodes.insert(rrt->nodes.end(), rrt->backTree.begin(), rrt->backTree.end());
        renderArea->update();
    }else{
        cout << "Error: check your rrtType which is: " << rrtType << endl;
    }
    auto duration = duration_cast<microseconds>(stop-start);
    cout << (success?"Successf":"Failed ")<< " to plan with elapsed time is: " << duration.count() << " microseconds" << endl;
    if(rrtType==3){
        if((rrtMult->nodeGroups.end()-1)->size()!=0){
            fout << (success?"s":"f") << " time: " << duration.count() << endl;
        }
    }else{
        fout << (success?"s":"f") << " time: " << duration.count() << endl;
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

void MainWindow::generateObstacles()
{
    if(this->isObsRandom){
        int sizeMin = 10, sizeMax = 80;
        int borderMin = 40, borderMax = 280;
        srand(time(0));

        for(int i=0;i<6;++i){
            int size = (rand()%(sizeMax-sizeMin))+sizeMin;
            Vector2f pos, pos2;
            pos[0] = (rand()%(borderMax-borderMin))+borderMin;
            pos[1] = (rand()%(borderMax-borderMin))+borderMin;
            pos2[0] = pos[0] + size;
            pos2[1] = pos[1] + size;
            this->renderArea->rrt->obstacles->addObstacle(pos,pos2);
            this->renderArea->rrtMult->obstacles->addObstacle(pos,pos2);
            // cout << pos[0] << " " << pos[1] << " " << pos2[0] << " " << pos2[1] << endl;
        }
    }else{
        vector<Vector2f> pointPos;
       pointPos.push_back(Vector2f(120,0));
       pointPos.push_back(Vector2f(130,140));

       pointPos.push_back(Vector2f(0,130));
       pointPos.push_back(Vector2f(80,140));

       pointPos.push_back(Vector2f(180,0));
       pointPos.push_back(Vector2f(190,250));

       pointPos.push_back(Vector2f(250,260));
       pointPos.push_back(Vector2f(400,270));

       pointPos.push_back(Vector2f(250,320));
       pointPos.push_back(Vector2f(260,400));

        // pointPos.push_back(Vector2f(120,43));
        // pointPos.push_back(Vector2f(191,114));

        // pointPos.push_back(Vector2f(87,76));
        // pointPos.push_back(Vector2f(114,103));

        for(size_t i=0;i<pointPos.size();i+=2){
            this->renderArea->rrt->obstacles->addObstacle(pointPos[i],pointPos[i+1]);
            this->renderArea->rrtMult->obstacles->addObstacle(pointPos[i],pointPos[i+1]);
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
