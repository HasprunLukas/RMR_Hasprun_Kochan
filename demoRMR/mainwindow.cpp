#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <fstream>
//#include <unistd.h>
//#include "robot.h"
//#include <tgmath.h>
///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU

double getTickToMeter(unsigned short previousTick, unsigned short tick);

void printGrid(int x, int y);
void printMatrix(int x, int y);
void mapCreator();
double xZelana = 5.25;//5.25
double yZelana = 2.30;//2.30
bool turningLeft = false;
bool turningRight = false;
bool isCorrectAngle = false;
bool isCorrectPosition = false;
bool startApp = true;
bool isRotatingR = false;
bool isRotatingL = false;
bool ismovingF = false;
bool ismovingB = false;
bool firstRun = true;
bool canContinue = true;
int speedT = 0;
double speedR = 0;
int speedDirection = 0;
int positionX = 0;
int positionY = 0;
double positionfi = 0;
cv::Mat myGrid(cv::Size(240, 240), CV_64F);
int grid[240][240] = {{0}};
int path[2][240] = {{0}};
double points[2][240] = {{0}};
int currPoint = 2;
double mapKoty[53][2] = {{0,0},{574.5,0},{574.5,460.5},{550.5,460.5},{550.5,471.5},{55.5,471.5},{55.5,431.5},{0,431.5},{0,0},//obvod
                  {264.5,0},{264.5,154.5},{267.5,154.5},{267.5,0},//1
                  {264.5,151.5},{264.5,154.5},{110,154.5},{110,151.5},//2
                  {110,151.5},{110,309},{114,309},{114,154.5},//3
                  {110,154.5},{110,309},{114,309},{114,154.5},//4
                  {574.5,309},{574.5,312},{365.5,312},{365.5,309},//5
                  {423,309},{423,154.5},{420,154.5},{420,309},//6
                  {423,154.50},{477.5,154.5},{477.5,157.5},{423,157.5},//7
                  {365.5,312},{365.5,366.5},{368.5,366.5},{368.5,312},//8
                  {267.5,254.5},{267.5,309},{264.5,309},{264.5,254.5},//9
                  {267.5,309},{267.5,312},{213,312},{213,309},//10
                  {213,309},{213,254.5},{216,254.5},{216,309}};//11

PositionData positionDataStruct;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
//    positionDataStruct.x = 0.5;
//    positionDataStruct.y = 0.5;
    positionDataStruct.x = 6.0;
    positionDataStruct.y = 6.0;
    positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
    positionY = positionDataStruct.y * 100;
    positionDataStruct.fi = 0;
    positionDataStruct.fi_radian = 0;
//    printMatrix(240, 240);
//    positionDataStruct.previousEncoderLeft = robotdata.EncoderLeft;
//    positionDataStruct.previousEncoderRight = robotdata.EncoderRight;

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
//    ipaddress="192.168.1.12";
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;




    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
//        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if(startApp) {
        positionDataStruct.previousEncoderLeft = robotdata.EncoderLeft;
        positionDataStruct.previousEncoderRight = robotdata.EncoderRight;

        startApp = false;
    }

    static double firstGyro=robotdata.GyroAngle;
    positionDataStruct.fi_gyro=(robotdata.GyroAngle-firstGyro)/100.0;
    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky

    long double d = 0.23; // vzdialenost medzi kolesami v metroch
    double lengthRight = getTickToMeter(positionDataStruct.previousEncoderRight, robotdata.EncoderRight);
    double lengthLeft = getTickToMeter(positionDataStruct.previousEncoderLeft, robotdata.EncoderLeft);
   // if(lengthRight == lengthLeft) {
//        positionDataStruct.fi_radian += (lengthRight - lengthLeft) / d;
//        positionDataStruct.fi = fmod((positionDataStruct.fi_radian)*(180/PI)+360.0,360.0);
//        positionDataStruct.fi = (robotdata.GyroAngle/100.0) + 180.0;
        positionDataStruct.fi = (robotdata.GyroAngle/100.0);
        if((robotdata.GyroAngle/100.0) < 0){
            positionDataStruct.fi = positionDataStruct.fi + 360.0;
        }
        positionDataStruct.x += ((lengthRight + lengthLeft)/2) * cos(positionDataStruct.fi_gyro*PI/180.0);
        positionDataStruct.y += ((lengthRight + lengthLeft)/2) * sin(positionDataStruct.fi_gyro*PI/180.0);

//        double wanted_angle = (atan2((yZelana - positionDataStruct.y),(xZelana - positionDataStruct.x))*(180/PI));
//        if(wanted_angle < 0) {
//        wanted_angle += 360;
//        }
//        cout << "wanted_angle: " << wanted_angle << endl;
//        positionDataStruct.fi_radian += (lengthRight - lengthLeft) / d;

//        positionDataStruct.fi_radian = (positionDataStruct.fi*PI/180.0);
//        positionDataStruct.fi = ((robotdata.GyroAngle/100.0)+180.0);
   /* } else {
        double previousFi = positionDataStruct.fi_radian;
//        positionDataStruct.fi_radian += (lengthRight - lengthLeft) / d;
        positionDataStruct.fi_radian += (lengthRight - lengthLeft) / d;
        positionDataStruct.fi = fmod((positionDataStruct.fi_radian)*(180/PI)+360.0,360.0);
//        positionDataStruct.fi_radian = (positionDataStruct.fi*PI/180.0);
//        positionDataStruct.fi = ((robotdata.GyroAngle/100.0)+180.0);
        positionDataStruct.x += ((d*(lengthRight+lengthLeft))/(2.0*(lengthRight-lengthLeft)))*(sin(positionDataStruct.fi_radian)-sin(previousFi));
        positionDataStruct.y -= ((d*(lengthRight+lengthLeft))/(2.0*(lengthRight-lengthLeft)))*(cos(positionDataStruct.fi_radian)-cos(previousFi));
    }*/
    positionDataStruct.previousEncoderLeft = robotdata.EncoderLeft;
    positionDataStruct.previousEncoderRight = robotdata.EncoderRight;

//    if(firstRun == true){
//        mapCreator();

////        ofstream myfile("/home/pocitac3/Documents/RMR_Uloha_1/PerfeknaMapa_predRozsirenimStien.txt");
////        ofstream myfile("C:/Users/Lenovo/OneDrive/Dokumenty/RMR/RMR_Uloha_1/PerfeknaMapa_predRozsirenimStien.txt");
//        ofstream myfile("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/PerfeknaMapa_predRozsirenimStien.txt");
//        if (myfile.is_open()){
//            myfile << "Here is your map! " << endl;
//            for (int i = 0; i < 240; i++)
//            {
//                myfile << "" << endl;
//                for (int j = 0; j < 240; j++){
//                    myfile << " " << grid[i][j];
//                }
//            }
//            myfile.close();
//        }
//        else cout << "Unable to open file";

//        /*
//         * Uloha 4
//         */
//        executeTask4(); //tato uloha bi sa mala spustit len raz na zaciatku celeho procesu a potom uz len pracovat s maticou suradnic trasi ktoru vytvorila, ak bi sa spustila znou trasa a aj zaplavovy algoritmus bi sa prepisali v zmisle aktualnej pozicie robota ako startovacia pozicia.
//        trajectory_run();
//        firstRun = false;
//    }
    /*
     * Uloha 3
     */
//    executeTask3(/*copyOfLaserData*/);
    /*
     * Uloha 1
     */
//    executeTask1(2.2, 0.45);
//    if(firstRun) {
//        trajectory_run();
//        firstRun = false;
//    }

//    double wantedX = (points[0][currPoint]*5)/100.0;
//    double wantedY = (points[1][currPoint]*5)/100.0;
//    cout << "xZelana : " << wantedX << endl;
//    cout << "yZelana : " << wantedY << endl;
//    if(canContinue) {
//        executeTask1(wantedX, wantedY);
//        if(isCorrectPosition) {
//            currPoint++;
//            if(points[0][currPoint] != 0 && points[1][currPoint] != 0) {
//                wantedX = (points[0][currPoint]*5)/100.0;
//                wantedY = (points[1][currPoint]*5)/100.0;
//                cout << "xZelana : " << wantedX << endl;
//                cout << "yZelana : " << wantedY << endl;
//                isCorrectPosition = false;
//                isCorrectAngle = false;
//            } else {
//                canContinue = false;
//            }
//        }
//    }




///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    if(datacounter%5)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
//        emit uiValuesChanged(robotdata.EncoderLeft,11,15);
        emit uiValuesChanged(positionDataStruct.x,positionDataStruct.y,positionDataStruct.fi);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    return 0;

}

void MainWindow::executeTask1(double xZelana, double yZelana) {
//    cout << "xZelana : " << xZelana << endl;
//    cout << "yZelana : " << yZelana << endl;
    cout << "xaktualna : " << positionDataStruct.x << endl;
    cout << "yaktualna : " << positionDataStruct.y << endl;
    double wanted_angle = atan2((yZelana - positionDataStruct.y),(xZelana - positionDataStruct.x))*(180/PI);
    if(wanted_angle < 0) {
        wanted_angle += 360;
    }
    cout << "angle : " << positionDataStruct.fi << endl;
    cout << "wanted_angle : " << wanted_angle << endl;
    if(!isCorrectAngle) {
        if(abs(wanted_angle - positionDataStruct.fi) < 1.0) {
            robot.setRotationSpeed(0);
            isRotatingR = false;
            isRotatingL = false;
            isCorrectAngle = true;
        } else {
            double rotation_speed = 3.14159/4;
            cout << "abs(wanted_angle - positionDataStruct.fi) : " << abs(wanted_angle - positionDataStruct.fi) << endl;
            if(abs(wanted_angle - positionDataStruct.fi) <= 30) {
                rotation_speed = (abs(wanted_angle - positionDataStruct.fi)*(PI/180)) * ((67.5*(PI/180))/(3.14159/4)); //((67.5*(PI/180))/(3.14159/4))=1.5
            }
//            cout << "Rotation speed: " << rotation_speed <<" "<<abs(wanted_angle - positionDataStruct.fi)<< endl;
            if(rotation_speed < 0.2) {
                rotation_speed = 0.2;
            }

            if((wanted_angle - positionDataStruct.fi) >= 0.0 && (wanted_angle - positionDataStruct.fi) < 180.0){
                robot.setRotationSpeed(rotation_speed); //turn left
                isRotatingR = false;
                isRotatingL = false;
            }
            else if((wanted_angle - positionDataStruct.fi) > 180.0){
                robot.setRotationSpeed(-rotation_speed); //turn right
                isRotatingR = false;
                isRotatingL = false;
            }
            else if((wanted_angle - positionDataStruct.fi) < 0.0 && (wanted_angle - positionDataStruct.fi) > -180.0){
                robot.setRotationSpeed(-rotation_speed); //turn right
                isRotatingR = false;
                isRotatingL = false;
            }
            else if((wanted_angle - positionDataStruct.fi) <= -180.0){
                robot.setRotationSpeed(rotation_speed); //turn left
                isRotatingR = false;
                isRotatingL = false;
            }
        }
    } else {
        if(abs(xZelana - positionDataStruct.x) < 0.05 && abs(yZelana - positionDataStruct.y) < 0.05) {
            isCorrectPosition = true;
            robot.setTranslationSpeed(0);
        } else if((xZelana + yZelana + 0.3) > (positionDataStruct.x + positionDataStruct.y) && (positionDataStruct.x + positionDataStruct.y) > (xZelana + yZelana - 0.3)){
            if(abs(wanted_angle - positionDataStruct.fi) < 1.0) {
                // prerobit na euklidovsku vzdialenost

                double speed = sqrt(pow((xZelana + yZelana), 2)+pow((positionDataStruct.x + positionDataStruct.y), 2)) * 1000; // priklad => 0.3*1000 = 300
//                double speed = abs((xZelana + yZelana) - (positionDataStruct.x + positionDataStruct.y)) * 1000; // priklad => 0.3*1000 = 300
                if(speed < 50) {
                    speed = 50;
                } else if(speed > 200) {
                    speed = 200;
                }
                robot.setTranslationSpeed(speed);
            } else {
                isCorrectAngle = false;
            }
        } else if(abs(wanted_angle - positionDataStruct.fi) > 1.0){
            isCorrectAngle = false;
        }else {
            robot.setTranslationSpeed(200);
        }
    }
}

double getTickToMeter(unsigned short previousTick, unsigned short tick) {
    double tickToMeter = 0.000085292090497737556558;
    double res = ((double)tick - (double)previousTick);
    if(res > 5000) {
        res = (long double)(tick-65536) - (long double)previousTick;
    } else if(res < -5000) {
        res = (long double)(tick+65536)- (long double)previousTick;
    }
    return tickToMeter * res;
}

void MainWindow::trajectory_run() {
    fstream trasa;
    trasa.open("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/Trasa.txt");
    if(trasa.is_open()) {
        string sa;
        int j = 0;
        while (getline(trasa, sa)) {
            // Print the data of the string.
            if(sa.size() > 0) {
                string delimiter = " ";
                size_t pos = 0;
                string token;
                int i = 0;
                while ((pos = sa.find(delimiter)) != string::npos) {
                    token = sa.substr(0, pos);
                    if(i > 0) {
//                        cout << "token " << i+1 << " : " << token << endl;
//                        cout << "j : " << j << ", i : " << i << endl;
                        points[j][i] = stod(token);
                    }
                    sa.erase(0, pos + delimiter.length());
                    i++;
                }
                j++;
            }
//            cout << "SIZE : " << sa.size() << endl;
//            cout << sa << "\n";
        }
    }
    trasa.close();
    cout << "path arr created" << endl;

//    for(int j = 0; j < 2; j++) {
//        for(int i = 0; i < 240; i++) {
//            cout << points[j][i] << " ";
//        }
//        cout << endl;
//    }

//    for(int i = 0; i < 240; i++) {
//        if(points[0][i] != 0 && points[1][i] != 0) {
//            cout << "ExecuteTask1(x=" << (points[0][i]*5)/100.0 << ", y=" << (points[1][i]*5)/100.0 << ");" << endl;
//        }
//    }

//    ofstream rozsirena("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/PerfeknaMapa_poRozsireni.txt");
//    if (rozsirena.is_open()){
//        for (int i = 0; i < 240; i++)
//        {
//            rozsirena << "" << endl;
//            for (int j = 0; j < 240; j++){
//                rozsirena << " " << grid[i][j];
//            }
//        }
//        rozsirena.close();
//    }
}

void MainWindow::executeTask2() {
    // look at file /RMR_Uloha_1/imageeeeeeeee.png, darker point is obstacle, brighter point is obstacle edge
    calculateObstacleEdges();
}

void MainWindow::calculateObstacleEdges() {
    double previousXg = NAN, previousYg = NAN, firstXg = NAN, firstYg = NAN;
    double threshold = 20.0;
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        if(copyOfLaserData.Data[k].scanDistance/1000.0 > 3 || copyOfLaserData.Data[k].scanDistance/1000.0 < 0.3 || (copyOfLaserData.Data[k].scanDistance/1000.0 > 0.63 && copyOfLaserData.Data[k].scanDistance/1000.0 < 0.71)) continue;
        double xg = 100*(positionDataStruct.x + ((copyOfLaserData.Data[k].scanDistance/1000.0)*cos((positionDataStruct.fi_gyro -copyOfLaserData.Data[k].scanAngle)*PI/180.0)));
        double yg = 100*(positionDataStruct.y + ((copyOfLaserData.Data[k].scanDistance/1000.0)*sin((positionDataStruct.fi_gyro-copyOfLaserData.Data[k].scanAngle)*PI/180.0)));
        if(k == 0) {
            if(copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanDistance/1000.0 > 3 || copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanDistance/1000.0 < 0.3 || (copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanDistance/1000.0 > 0.63 && copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanDistance/1000.0 < 0.71)) continue;
            previousXg = 100*(positionDataStruct.x + ((copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanDistance/1000.0)*cos((positionDataStruct.fi_gyro -copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanAngle)*PI/180.0)));
            previousYg = 100*(positionDataStruct.y + ((copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanDistance/1000.0)*sin((positionDataStruct.fi_gyro-copyOfLaserData.Data[copyOfLaserData.numberOfScans-1].scanAngle)*PI/180.0)));
            firstXg = xg;
            firstYg = yg;
        }

        double distBetweenPoints = sqrt(pow((xg-previousXg), 2) + pow((yg-previousYg), 2));

        if(distBetweenPoints <= threshold) {
            cout << "sqrt(pow((" << xg << "-" << previousXg << "), 2) + pow((" << yg << "-" << previousYg << "), 2))=" << distBetweenPoints << endl;
            //            myGrid.at<double>((int) (yg/5.0), (int) (xg/5.0)) = 255;
            cout << "Iteration " << k+1 << ", Distance between points (xg,yg)=" << xg << "," << yg << " and (previousXg,previousYg)=" << previousXg << "," << previousYg << " is " << distBetweenPoints << endl;
            myGrid.at<double>((int) (yg/5.0), (int) (xg/5.0)) = 120;
        } else if(distBetweenPoints > threshold) {
            myGrid.at<double>((int) (previousYg/5.0), (int) (previousXg/5.0)) = 255;
            myGrid.at<double>((int) (yg/5.0), (int) (xg/5.0)) = 255;
        }

        if(k == copyOfLaserData.numberOfScans - 1) {
            distBetweenPoints = sqrt(pow((xg-firstXg), 2) + pow((yg-firstYg), 2));
            if(distBetweenPoints > threshold) {
                myGrid.at<double>((int) (yg/5.0), (int) (xg/5.0)) = 255;
                myGrid.at<double>((int) (firstYg/5.0), (int) (firstXg/5.0)) = 255;
            }
        }

        if(k > 0) {
            previousXg = xg;
            previousYg = yg;
        }
        //            if(k == 0) {
        //                cout<<"xg = " << xg << endl;
        //                cout<<"yg = " << yg << endl;
        //            }
        //        grid[(int) (yg/5.0)][(int) (xg/5.0)] = 1;
        //        myGrid.at<double>((int) (yg/5.0),(int) (xg/5.0)) = 255;
        //        cout << "Coordinates for point " << k+1 << " -> xg : " << xg << ", yg : " << yg << endl;
    }
}

void MainWindow::executeTask3(/*LaserMeasurement copyOfLaserData*/) {
        if((!isRotatingR && !isRotatingL) && (speedDirection == 1 || speedDirection == -1 )) {
//            if(speedT >= 50 && speedT <= 200) {
//                for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
//                {
//                    if(copyOfLaserData.Data[k].scanDistance/1000.0 > 3 || copyOfLaserData.Data[k].scanDistance/1000.0 < 0.3 || (copyOfLaserData.Data[k].scanDistance/1000.0 > 0.63 && copyOfLaserData.Data[k].scanDistance/1000.0 < 0.71)) continue;
//                    double xg = 100*(positionDataStruct.x + ((copyOfLaserData.Data[k].scanDistance/1000.0)*cos(positionDataStruct.fi_radian + (-copyOfLaserData.Data[k].scanAngle*PI/180.0))));
//                    double yg = 100*(positionDataStruct.y + ((copyOfLaserData.Data[k].scanDistance/1000.0)*sin(positionDataStruct.fi_radian + (-copyOfLaserData.Data[k].scanAngle*PI/180.0))));
//        //            if(k == 0) {
//        //                cout<<"xg = " << xg << endl;
//        //                cout<<"yg = " << yg << endl;
//        //            }
//                    grid[(int) (yg/5.0)][(int) (xg/5.0)] = 1;
//                    myGrid.at<double>((int) (yg/5.0),(int) (xg/5.0)) = 255;
//                }
//            }

            if(ismovingF == true){
                double absolut_distance = sqrt(pow(((positionDataStruct.x * 100.0) - positionX), 2) + pow(((positionDataStruct.y * 100.0) - positionY), 2)); // to  * 100 is to transform m into cm
                speedT = (absolut_distance / 50) * 200;
                if(speedT < 50){
                    speedT = 50;
                    robot.setTranslationSpeed(speedT);
                }
                else if(speedT >= 200){
                    speedT = 200;
                    robot.setTranslationSpeed(speedT);
                }
                else{
                    robot.setTranslationSpeed(speedT);
                }
                speedDirection = 1;
                cout << "absolut_distance: " << absolut_distance << endl;
                cout << "setTranslationSpeed: " << (speedT) << endl;
                cout << "-------------------------" << endl;

            }
            else if(ismovingB == true){
                double absolut_distance = sqrt(pow(((positionDataStruct.x * 100.0) - positionX), 2) + pow(((positionDataStruct.y * 100.0) - positionY), 2)); // to  * 100 is to transform m into cm
                speedT = (absolut_distance / 50) * 250;
                if(speedT < 50){
                    speedT = 50;
                    robot.setTranslationSpeed(-speedT);
                }
                else if(speedT >= 250){
                    speedT = 250;
                    robot.setTranslationSpeed(-speedT);
                }
                else{
                    robot.setTranslationSpeed(-speedT);
                }
                speedDirection = -1;
                cout << "absolut_distance: " << absolut_distance << endl;
                cout << "setTranslationSpeed: " << (speedT) << endl;
                cout << "-------------------------" << endl;
            }
            else if((ismovingF == false) && (speedDirection == 1)){
                double absolut_distance = (speedT * 50.0) / 200.0;
                speedT = ((absolut_distance - 0.2) / 50) * 200;
                if(speedT <= 0){
                    speedT = 0;
                    robot.setTranslationSpeed(speedT);
                }
                else{
                    robot.setTranslationSpeed(speedT);
                }
                cout << "absolut_distance: " << absolut_distance << endl;
                cout << "setTranslationSpeed: " << (speedT) << endl;
                cout << "-------------------------" << endl;
            }
            else if((ismovingB == false) && (speedDirection == -1)){
                double absolut_distance = (speedT * 50.0) / 250.0;
                speedT = ((absolut_distance - 0.2) / 50) * 250;
                if(speedT <= 0){
                    speedT = 0;
                    robot.setTranslationSpeed(-speedT);
                }
                else{
                    robot.setTranslationSpeed(-speedT);
                }
                cout << "absolut_distance: " << absolut_distance << endl;
                cout << "setTranslationSpeed: " << (speedT) << endl;
                cout << "-------------------------" << endl;
            }

//            cout << "speedDirection: " << (speedDirection) << endl;
            if(speedT == 0){
                speedDirection = 0;
                positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
                positionY = positionDataStruct.y * 100;
                positionfi = positionDataStruct.fi;
            }
//            cv::imwrite("C:/Users/Lenovo/OneDrive/Dokumenty/RMR/RMR_Uloha_1/imageeeeeeeee.png", myGrid);
//            cv::imwrite("/home/pocitac3/Documents/RMR_Uloha_1/imageeeeeeeee.png", myGrid);
        }
        /* ak bi som mal ze positionfi je -+180
        if(positionfi < 0){
            positionfi = positionfi + 360;
        }
        */
        else if((!ismovingF && !ismovingB) && (speedDirection == 2 || speedDirection == -2 )){ //rotating right and left
            if(isRotatingR == true){
                double absolut_angle = positionfi - positionDataStruct.fi; // absolut_angle is in degrees and speedT is in radians
                if(absolut_angle < 0){
                    absolut_angle += 360;
                }
                speedR = ((absolut_angle / 45.0) * 45.0) * (PI/180.0); // x = a*(pi/180) degrees into radians. first(number) 45 = pi/4 -> max speedR and it will rich it after secont(number) 45 deg.
                if(speedR < 0.2){
                    speedR = 0.2;
                    robot.setRotationSpeed(-speedR);
                }
                else if(speedR >= PI/4.0){
                    speedR = PI/4.0;
                    robot.setRotationSpeed(-speedR);
                }
                else{
                    robot.setRotationSpeed(-speedR);
                }
                speedDirection = 2;
                cout << "absolut_angle: " << absolut_angle << endl;
                cout << "setRotationSpeed: " << (speedR) << endl;
                cout << "-------------------------" << endl;

            }
            else if(isRotatingL == true){
                double absolut_angle = positionDataStruct.fi - positionfi; // absolut_angle is in degrees and speedT is in radians
                if(absolut_angle < 0){
                    absolut_angle += 360;
                }
                speedR = ((absolut_angle / 45.0) * 45.0) * (PI/180.0); // x = a*(pi/180) degrees into radians. first(number) 45 = pi/4 -> max speedR and it will rich it after secont(number) 45 deg.
                if(speedR < 0.2){
                    speedR = 0.2;
                    robot.setRotationSpeed(speedR);
                }
                else if(speedR >= PI/4.0){
                    speedR = PI/4.0;
                    robot.setRotationSpeed(speedR);
                }
                else{
                    robot.setRotationSpeed(speedR);
                }
                speedDirection = -2;
                cout << "absolut_angle: " << absolut_angle << endl;
                cout << "setRotationSpeed: " << (speedR) << endl;
                cout << "-------------------------" << endl;
            }
            else if((isRotatingR == false) && (speedDirection == 2)){
                double absolut_angle = ((speedR * (180.0/PI)) * 45.0) / 45.0; // absolut_angle is in degrees and speedT is in radians
                speedR = (((absolut_angle - 1.0) / 45.0) * 45.0) * (PI/180.0);
                if(speedR <= 0.0){
                    speedR = 0.0;
                    robot.setRotationSpeed(-speedR);
                }
                else{
                    robot.setRotationSpeed(-speedR);
                }
                cout << "absolut_angle: " << absolut_angle << endl;
                cout << "setRotationSpeed: " << (speedR) << endl;
                cout << "-------------------------" << endl;
            }
            else if((isRotatingL == false) && (speedDirection == -2)){
                double absolut_angle = ((speedR * (180.0/PI)) * 45.0) / 45.0; // absolut_angle is in degrees and speedT is in radians
                speedR = (((absolut_angle - 1.0) / 45.0) * 45.0) * (PI/180.0);
                if(speedR <= 0.0){
                    speedR = 0.0;
                    robot.setRotationSpeed(speedR);
                }
                else{
                    robot.setRotationSpeed(speedR);
                }
                cout << "absolut_angle: " << absolut_angle << endl;
                cout << "setRotationSpeed: " << (speedR) << endl;
                cout << "-------------------------" << endl;
            }

//            cout << "speedDirection: " << (speedDirection) << endl;
            if(speedR == 0.0){
                speedDirection = 0;
                positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
                positionY = positionDataStruct.y * 100;
                positionfi = positionDataStruct.fi; // v stupnoch
            }
        }
}

void MainWindow::executeTask4(){
//    int polomer_robota = 15;
    int matica = 0;

    for(int i = 0; i < 240; i++) { // rozsirenie hran stien
        for(int j = 0; j < 240; j++) {
            matica = grid[i][j];
            if(matica == 1){
                for(int k = 1; k <= 5; k++){ //k=3 lebo 5cm * 3 = 15 cm je polomer robota
                    for(int m = i - 5; m <= (i + 5); m++){
                        for(int n = j - 5; n <= (j + 5); n++){
                            if((m >= 0 && m < 240) && (n >= 0 && n < 240) && (grid[m][n] == 0)) // ak maica necacina od 0 a konci v 239 treva zmenit podmienky
                                grid[m][n] = 3;
                        }
                    }
                }
            }
        }
    }

    for(int i = 0; i < 240; i++) { // rozsirenie hran stien cast 2
        for(int j = 0; j < 240; j++) {
            matica = grid[i][j];
            if(matica == 3){
                grid[i][j] = 1;
            }
        }
    }

    ofstream rozsirena("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/PerfeknaMapa_poRozsireni.txt");
    if (rozsirena.is_open()){
        for (int i = 0; i < 240; i++)
        {
            rozsirena << "" << endl;
            for (int j = 0; j < 240; j++){
                rozsirena << " " << grid[i][j];
            }
        }
        rozsirena.close();
    }
    else cout << "Unable to open file";


    grid[(int) (xZelana * 100)/5][(int) (yZelana * 100)/5] = 2; // position of finish
    int startX = (int) (positionDataStruct.x * 100.0)/5.0; // starting position x in cm
    int startY = (int) (positionDataStruct.y * 100.0)/5.0; // starting position y in cm


    for(int k = 2; !(grid[startX][startY] > 2); k++){ // zaplavovi algoritmus

        for(int i = 0; i < 240; i++) {
            for(int j = 0; j < 240; j++) {
                matica = grid[i][j];
                if(matica == k){
                    if((i+1 >= 0 && i+1 < 240) && (j >= 0 && j < 240) && grid[i+1][j] == 0)
                        grid[i+1][j] = k+1;

                    if((i-1 >= 0 && i-1 < 240) && (j >= 0 && j < 240) && grid[i-1][j] == 0)
                        grid[i-1][j] = k+1;

                    if((i >= 0 && i < 240) && (j+1 >= 0 && j+1 < 240) && grid[i][j+1] == 0)
                        grid[i][j+1] = k+1;

                    if((i >= 0 && i < 240) && (j-1 >= 0 && j-1 < 240) && grid[i][j-1] == 0)
                        grid[i][j-1] = k+1;
                }
            }
        }
    }

    ofstream mapa("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/PerfeknaMapa.txt");
    if (mapa.is_open()){
        for (int i = 0; i < 240; i++)
        {
            mapa << "" << endl;
            for (int j = 0; j < 240; j++){
                if(grid[i][j] < 10) mapa << "  ";
                else if(grid[i][j] < 100) mapa << " ";
                mapa << " " << grid[i][j];
            }
        }
        mapa.close();
    }
    else cout << "Unable to open file";

    path[0][0] = startX;
    path[1][0] = startY;
    //najst cestu od zaciatku po ciel (mnozina bodov)
    for(int controled_position = 0, i = startX, j = startY, k = 0, direction = 0; !(controled_position == 2); i = path[0][k], j = path[1][k]) {

        if((i+1 >= 0 && i+1 < 240) && grid[i+1][j] == (grid[i][j]) - 1){ // down
            if (direction == 1){
                path[0][k] = i + 1;
                path[1][k] = j;
                direction = 1;
            }
            else{
                path[0][k+1] = i + 1;
                path[1][k+1] = j;
                direction = 1;
                k++;
            }

            controled_position = grid[i+1][j];
        }
        else if((j+1 >= 0 && j+1 < 240) && grid[i][j+1] == (grid[i][j]) - 1){ // right
            if (direction == 2){
                path[0][k] = i;
                path[1][k] = j + 1;
                direction = 2;
            }
            else{
                path[0][k+1] = i;
                path[1][k+1] = j + 1;
                direction = 2;
                k++;
            }

            controled_position = grid[i][j+1];
        }
        else if((i-1 >= 0 && i-1 < 240) && grid[i-1][j] == (grid[i][j]) - 1){ //up
            if (direction == 3){
                path[0][k] = i - 1;
                path[1][k] = j;
                direction = 3;
            }
            else{
                path[0][k+1] = i - 1;
                path[1][k+1] = j;
                direction = 3;
                k++;
            }

            controled_position = grid[i-1][j];
        }
        else if((j-1 >= 0 && j-1 < 240) && grid[i][j-1] == (grid[i][j]) - 1){ // left
            if (direction == 4){
                path[0][k] = i;
                path[1][k] = j - 1;
                direction = 4;
            }
            else{
                path[0][k+1] = i;
                path[1][k+1] = j - 1;
                direction = 4;
                k++;
            }

            controled_position = grid[i][j-1];
        }
    }

 ofstream trasa("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/Trasa.txt");
 if (trasa.is_open()){
     for (int i = 0; i < 2; i++)
     {
         trasa << "" << endl;
         for (int j = 0; j < 240; j++){
             trasa << " " << path[i][j];
         }
     }
     trasa.close();
 }
 else cout << "Unable to open file";
}

void mapCreator(){
    for(int i = 0; i < 52; i++){
        if(mapKoty[i][0] == mapKoty[i+1][0]){//suradnice x
            int j = 0;
            if(mapKoty[i][1] > mapKoty[i+1][1]){
                for(j = mapKoty[i][1]; j > mapKoty[i+1][1]; j--){
                    grid[(int) (mapKoty[i][0]) / 5][(int) j / 5] = 1;
                }
            }
            else{
                for(j = mapKoty[i][1]; j < mapKoty[i+1][1]; j++){
                    grid[(int) (mapKoty[i][0]) / 5][(int) j / 5] = 1;
                }
            }
        }
        else if(mapKoty[i][1] == mapKoty[i+1][1]){//suradnice y
            int j = 0;
            if(mapKoty[i][0] > mapKoty[i+1][0]){
                for(j = mapKoty[i][0]; j > mapKoty[i+1][0]; j--){
                    grid[(int) j / 5][(int) (mapKoty[i][1]) / 5] = 1;
                }
            }
            else{
                for(j = mapKoty[i][0]; j < mapKoty[i+1][0]; j++){
                    grid[(int) j / 5][(int) (mapKoty[i][1]) / 5] = 1;
                }
            }
        }
        else{//sikme steny
            if(i%4 != 0){
                double Dis = sqrt(pow((mapKoty[i+1][0] - mapKoty[i][0]), 2) + pow((mapKoty[i+1][1] - mapKoty[i][1]), 2));
                double Ux = (mapKoty[i+1][0] - mapKoty[i][0]) / Dis;
                double Uy = (mapKoty[i+1][1] - mapKoty[i][1]) / Dis;

                for (int l = 1; (sqrt(pow((((Ux * l) + mapKoty[i][0]) - mapKoty[i][0]), 2) + pow((((Uy * l) + mapKoty[i][1]) - mapKoty[i][1]), 2))) < Dis ; l++){
                    grid[(int) (((Ux * l) + mapKoty[i][0]) / 5)][(int) (((Uy * l) + mapKoty[i][1]) / 5)] = 1;
                }
            }
        }
    }
}

void printGrid(int x, int y) {
    for(int i = 0; i < y; i++) {
        for(int j = 0; j < x; j++) {
            cout << grid[i][j] << " ";
        }
        cout << endl;
    }
}

void printMatrix(int x, int y) {
    for(int i = 0; i < y; i++) {
        for(int j = 0; j < x; j++) {
//            cout << grid[i][j] << " ";
            cout << myGrid.at<double>(i,j) << " ";
        }
        cout << endl;
    }
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
//    if(speedT >= 50 && speedT <= 200) {
//        for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
//        {
//            if(copyOfLaserData.Data[k].scanDistance/1000.0 > 3 || copyOfLaserData.Data[k].scanDistance/1000.0 < 0.3 || (copyOfLaserData.Data[k].scanDistance/1000.0 > 0.63 && copyOfLaserData.Data[k].scanDistance/1000.0 < 0.71)) continue;
//            double xg = 100*(positionDataStruct.x + ((copyOfLaserData.Data[k].scanDistance/1000.0)*cos((positionDataStruct.fi_gyro -copyOfLaserData.Data[k].scanAngle)*PI/180.0)));
//            double yg = 100*(positionDataStruct.y + ((copyOfLaserData.Data[k].scanDistance/1000.0)*sin((positionDataStruct.fi_gyro-copyOfLaserData.Data[k].scanAngle)*PI/180.0)));
////            if(k == 0) {
////                cout<<"xg = " << xg << endl;
////                cout<<"yg = " << yg << endl;
////            }
//            grid[(int) (yg/5.0)][(int) (xg/5.0)] = 1;
//            myGrid.at<double>((int) (yg/5.0),(int) (xg/5.0)) = 255;
//        }
//    }
    executeTask2();
    cv::imwrite("C:/Users/haspr/Documents/School/RMR/RMR_Uloha_1/imageeeeeeeee.png", myGrid);
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
//    cout<<"W: " << cameraData.size().width<< endl;
//    cout<<"H: " << cameraData.size().height<< endl;
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    robot.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    //ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(300);
//    robot.setTranslationSpeed(speedT);
//    if(speedDirection == 0){
//        ismovingF = true;
//        ismovingB = false;
//        isRotatingR = false;
//        isRotatingL = false;
//        speedDirection = 1;
//        positionfi = positionDataStruct.fi; // v stupnoch
//    }

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);
//    robot.setTranslationSpeed(speedT);
//    if(speedDirection == 0){
//        ismovingF = false;
//        ismovingB = true;
//        isRotatingR = false;
//        isRotatingL = false;
//        speedDirection = -1;
//        positionfi = positionDataStruct.fi; // v stupnoch
//    }

}

void MainWindow::on_pushButton_6_clicked() //left
{
    robot.setRotationSpeed(3.14159/2);
//    if(speedDirection == 0){
//      //  positionDataStruct.fi = ((robotdata.GyroAngle/100.0)+180.0);
//        ismovingF = false;
//        ismovingB = false;
//        isRotatingR = false;
//        isRotatingL = true;
//        speedDirection = -2;
//        positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
//        positionY = positionDataStruct.y * 100;
//        speedT = 0;
//    }
}

void MainWindow::on_pushButton_5_clicked()//right
{
    robot.setRotationSpeed(-3.14159/2);
//    if(speedDirection == 0){
//      //  positionDataStruct.fi = ((robotdata.GyroAngle/100.0)+180.0);
//        ismovingF = false;
//        ismovingB = false;
//        isRotatingR = true;
//        isRotatingL = false;
//        speedDirection = 2;
//        positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
//        positionY = positionDataStruct.y * 100;
//        speedT = 0;
//    }

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);

//    ismovingF = false;
//    ismovingB = false;
//    isRotatingR = false;
//    isRotatingL = false;
//    positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
//    positionY = positionDataStruct.y * 100;
//    positionfi = positionDataStruct.fi; // v stupnoch


}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}
