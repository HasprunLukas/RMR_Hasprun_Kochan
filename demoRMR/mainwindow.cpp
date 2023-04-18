#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
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
double xZelana = -3.0;
double yZelana = -3.0;
bool turningLeft = false;
bool turningRight = false;
bool isCorrectAngle = false;
bool isCorrectPosition = false;
bool startApp = true;
bool isRotating = false;
bool ismovingF = false;
bool ismovingB = false;
int speedT = 0;
int positionX = 0;
int positionY = 0;
cv::Mat myGrid(cv::Size(240, 240), CV_64F);
int grid[240][240] = {{0}};

PositionData positionDataStruct;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    positionDataStruct.x = 6;
    positionDataStruct.y = 6;
    positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
    positionY = positionDataStruct.y * 100;
    positionDataStruct.fi = 0;
    positionDataStruct.fi_radian = 0;
    printMatrix(240, 240);
//    positionDataStruct.previousEncoderLeft = robotdata.EncoderLeft;
//    positionDataStruct.previousEncoderRight = robotdata.EncoderRight;

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
//    ipaddress="127.0.0.1";
    ipaddress="192.168.1.14";
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
        std::cout<<actIndex<<std::endl;
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

    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky

    long double d = 0.23; // vzdialenost medzi kolesami v metroch
    double lengthRight = getTickToMeter(positionDataStruct.previousEncoderRight, robotdata.EncoderRight);
    double lengthLeft = getTickToMeter(positionDataStruct.previousEncoderLeft, robotdata.EncoderLeft);
    if(lengthRight == lengthLeft) {
        positionDataStruct.x += ((lengthRight + lengthLeft)/2) * cos(positionDataStruct.fi_radian);
        positionDataStruct.y += ((lengthRight + lengthLeft)/2) * sin(positionDataStruct.fi_radian);
//        positionDataStruct.fi_radian += (lengthRight - lengthLeft) / d;
        positionDataStruct.fi_radian += fmod(((lengthRight - lengthLeft) / d)+2*PI, 2*PI);
        positionDataStruct.fi = fmod((positionDataStruct.fi_radian)*(180/PI)+360.0,360.0);
    } else {
        double previousFi = positionDataStruct.fi_radian;
//        positionDataStruct.fi_radian += (lengthRight - lengthLeft) / d;
        positionDataStruct.fi_radian += fmod(((lengthRight - lengthLeft) / d)+2*PI, 2*PI);
        positionDataStruct.fi = fmod((positionDataStruct.fi_radian)*(180/PI)+360.0,360.0);
        positionDataStruct.x += (d*(lengthRight+lengthLeft))/(2*(lengthRight-lengthLeft))*(sin(positionDataStruct.fi_radian)-sin(previousFi));
        positionDataStruct.y -= (d*(lengthRight+lengthLeft))/(2*(lengthRight-lengthLeft))*(cos(positionDataStruct.fi_radian)-cos(previousFi));
    }
    positionDataStruct.previousEncoderLeft = robotdata.EncoderLeft;
    positionDataStruct.previousEncoderRight = robotdata.EncoderRight;

    /*
     * Uloha 3
     */
    executeTask3(copyOfLaserData);
    /*
     * Uloha 1
     */
//    executeTask1();




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

void MainWindow::executeTask1() {
    double wanted_angle = atan2((yZelana - positionDataStruct.y),(xZelana - positionDataStruct.x))*(180/PI);
    if(wanted_angle < 0) {
        wanted_angle += 360;
    }
//    cout << "wanted_angle: " << wanted_angle << endl;
    if(!isCorrectAngle) {
        if(abs(wanted_angle - positionDataStruct.fi) < 2.0) {
            robot.setRotationSpeed(0);
            isRotating = false;
            isCorrectAngle = true;
        } else {
            double rotation_speed = 3.14159/4;
            if(abs(wanted_angle - positionDataStruct.fi) <= 30) {
                rotation_speed = (abs(wanted_angle - positionDataStruct.fi)*(PI/180)) * ((67.5*(PI/180))/(3.14159/4));
            }
//            cout << "Rotation speed: " << rotation_speed <<" "<<abs(wanted_angle - positionDataStruct.fi)<< endl;
            if(rotation_speed < 0.2) {
                rotation_speed = 0.2;
            }

            if((wanted_angle - positionDataStruct.fi) >= 0.0 && (wanted_angle - positionDataStruct.fi) < 180.0){
                robot.setRotationSpeed(rotation_speed); //turn left
                isRotating = true;
            }
            else if((wanted_angle - positionDataStruct.fi) > 180.0){
                robot.setRotationSpeed(-rotation_speed); //turn right
                isRotating = true;
            }
            else if((wanted_angle - positionDataStruct.fi) < 0.0 && (wanted_angle - positionDataStruct.fi) > -180.0){
                robot.setRotationSpeed(-rotation_speed); //turn right
                isRotating = true;
            }
            else if((wanted_angle - positionDataStruct.fi) < -180.0){
                robot.setRotationSpeed(rotation_speed); //turn left
                isRotating = true;
            }
        }
    } else {
        if(abs(xZelana - positionDataStruct.x) < 0.05 && abs(yZelana - positionDataStruct.y) < 0.05) {
            isCorrectPosition = true;
            robot.setTranslationSpeed(0);
        } else if((xZelana + yZelana + 0.3) > (positionDataStruct.x + positionDataStruct.y) && (positionDataStruct.x + positionDataStruct.y) > (xZelana + yZelana - 0.3)){
            if(abs(wanted_angle - positionDataStruct.fi) < 2.0) {
                // prerobit na euklidovsku vzdialenost

                double speed = sqrt(pow((xZelana + yZelana), 2)+pow((positionDataStruct.x + positionDataStruct.y), 2)) * 1000; // priklad => 0.3*1000 = 300
//                double speed = abs((xZelana + yZelana) - (positionDataStruct.x + positionDataStruct.y)) * 1000; // priklad => 0.3*1000 = 300
                if(speed < 50) {
                    speed = 50;
                }
                robot.setTranslationSpeed(speed);
            } else {
                isCorrectAngle = false;
            }
        } else if(abs(wanted_angle - positionDataStruct.fi) > 2.0){
            isCorrectAngle = false;
        }else {
            robot.setTranslationSpeed(300);
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

void MainWindow::executeTask3(LaserMeasurement copyOfLaserData) {
        if(!isRotating) {
            if(speedT >= 50 && speedT <= 200) {
                for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
                {
                    if(copyOfLaserData.Data[k].scanDistance/1000.0 > 3 || copyOfLaserData.Data[k].scanDistance/1000.0 < 0.3) continue;
                    double xg = 100*(positionDataStruct.x + ((copyOfLaserData.Data[k].scanDistance/1000.0)*cos(positionDataStruct.fi_radian + (-copyOfLaserData.Data[k].scanAngle*PI/180.0))));
                    double yg = 100*(positionDataStruct.y + ((copyOfLaserData.Data[k].scanDistance/1000.0)*sin(positionDataStruct.fi_radian + (-copyOfLaserData.Data[k].scanAngle*PI/180.0))));
        //            if(k == 0) {
        //                cout<<"xg = " << xg << endl;
        //                cout<<"yg = " << yg << endl;
        //            }
                    grid[(int) (yg/5.0)][(int) (xg/5.0)] = 1;
                    myGrid.at<double>((int) (yg/5.0),(int) (xg/5.0)) = 255;
                }
            }

            if(ismovingF == true){
                double absolut_distance = sqrt(pow(((positionDataStruct.x * 100.0) - positionX), 2) + pow(((positionDataStruct.y * 100.0) - positionY), 2)); // to  * 100 is to transform m into cm
                speedT = (absolut_distance / 50) * 300;
                if(speedT > 300) speedT = 300;

                if(speedT < 50){
                    robot.setTranslationSpeed(50);
//                    speedT = 50;
                }
                else if(speedT >= 300){
                    robot.setTranslationSpeed(300);
//                    speedT = 300;
                }
                else{
                    robot.setTranslationSpeed(speedT);
//                    speedT = speedT;
                }
                cout << "absolut_distance: " << absolut_distance << endl;
                cout << "setTranslationSpeed: " << (speedT) << endl;
                cout << "-------------------------" << endl;

            }
            else if(ismovingB == true){
                double absolut_distance = sqrt(pow(((positionDataStruct.x * 100.0) - positionX), 2) + pow(((positionDataStruct.y * 100.0) - positionY), 2)); // to  * 100 is to transform m into cm
                speedT = (absolut_distance / 50) * 250;
                if(speedT > 250) speedT = 250;

                if(speedT < 50){
                    robot.setTranslationSpeed(-50);
//                    speedT = -50;
                }
                else if(speedT >= 250){
                    robot.setTranslationSpeed(-250);
//                    speedT = -250;
                }
                else{
                    robot.setTranslationSpeed(-speedT);
//                    speedT = -speedT;
                }
                cout << "absolut_distance: " << absolut_distance << endl;
                cout << "setTranslationSpeed: " << (speedT) << endl;
                cout << "-------------------------" << endl;
            }


    //        int k = 0;

    //        cout << "scanAngle : " << copyOfLaserData.Data[0].scanAngle << endl;
    //        cout << "scanDistance : " << copyOfLaserData.Data[0].scanDistance << endl;

    //        cout << "Retard" << endl;
    //        printGrid(240,240);
    //        printMatrix(240, 240);
            //TODO zmen cestu
            cv::imwrite("/home/pocitac3/Documents/RMR_Uloha_1/imageeeeeeeee.png", myGrid);
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
//    robot.setTranslationSpeed(300);
//    robot.setTranslationSpeed(speedT);
    ismovingF = true;
    ismovingB = false;
    isRotating = false;
}

void MainWindow::on_pushButton_3_clicked() //back
{
//    robot.setTranslationSpeed(-250);
//    robot.setTranslationSpeed(speedT);
    ismovingF = false;
    ismovingB = true;
    isRotating = false;
}

void MainWindow::on_pushButton_6_clicked() //left
{
    robot.setRotationSpeed(3.14159/2);
    ismovingF = false;
    ismovingB = false;
    isRotating = true;
    positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
    positionY = positionDataStruct.y * 100;
}

void MainWindow::on_pushButton_5_clicked()//right
{
    robot.setRotationSpeed(-3.14159/2);
    ismovingF = false;
    ismovingB = false;
    isRotating = true;
    positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
    positionY = positionDataStruct.y * 100;
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);
    ismovingF = false;
    ismovingB = false;
//    // true nech nesnima body pri STOP
    isRotating = true;
    positionX = positionDataStruct.x * 100; // to  * 100 is to transform m into cm
    positionY = positionDataStruct.y * 100;
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
