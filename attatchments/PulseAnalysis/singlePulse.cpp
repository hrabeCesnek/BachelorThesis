

#include "TFile.h"
#include "TH1.h"
#include "TF1.h"
#include "TGraph.h"
#include "TLine.h"
//#include "TTree.h"
#include "TCanvas.h"
#include "TMultiGraph.h"
#include <iostream>
#include <fstream>
#include <numeric>
 #include <TF1Convolution.h>


using namespace std;


#define no_deriv 1
#define time_step 2
#define min_slope 0.00005
#define min_time 20
#define max_time 110

#include "singlePulse.h"
//class Pulse {
//public:
double getError(vector<double> data){

double m =accumulate( data.begin(), data.end(), 0.0)/data.size();
double accum = 0;
  for (double d : data) {
    accum += (d - m) * (d - m);
  }
double error = sqrt(accum/(data.size() * (data.size()-1)));
return error;
}



  void Pulse::SetConditions(string fil){

    std::vector<double> temperatures; //= new vector<int>;
    //std::vector<double>  Source_voltage;
    std::vector<double> powers;
    string str = "nic";
    double temperature = -300;
    double power = -300;
    cout << fil << '\n';
    ifstream f(fil);

    getline(f,str);//skip first 4
    getline(f,str);
    getline(f,str);
    getline(f,str);
    while (getline(f,str))
    {
      cout << str << endl;
      str.erase(str.begin(), str.begin() + 21);
      //sscanf(str.c_str(),"%*i/%*i/%*i %*i:%*i:%*i,\t %lf,\t %lf,\t %*f",&power,&temperature);
      sscanf(str.c_str(),"%lf,\t %lf,\t %*f",&power,&temperature);
      std::cout << temperature << '\n';
      std::cout << power << '\n';
      temperatures.push_back(temperature);
      powers.push_back(power); //invert back

    }

    this->power = accumulate( powers.begin(), powers.end(), 0.0)/powers.size();

    this->PMT_temperature = accumulate( temperatures.begin(), temperatures.end(), 0.0)/temperatures.size();

    this->power_error = getError(powers);

    this->PMT_temperature_error = getError(temperatures);
  }


    Pulse::Pulse(string fil)
        {
          TCanvas *canvas_1;
          TCanvas *canvas_2;
          std::cout << fil << '\n';
          canvas_1 = new TCanvas("c1","full show", 700,500);
          canvas_1->SetLeftMargin(0.2);
          //FILE *fp = fopen("/home/dannezlomnyj/Documents/Programming/CalibrationSources/ReadCP/linux-build-files/data/1625173205.txt","r");
          ifstream f(fil);

        string str;
        //Int_t n = 0;
        Float_t time;
        Float_t voltage;
        std::vector<Float_t> times; //= new vector<int>;
        std::vector<Float_t>  voltages;//= new vector<double>;
        getline(f,str);//skip firs
        while (getline(f,str))
        {
          sscanf(str.c_str(),"%f,\t %f",&time,&voltage);
          //std::cout << voltage << '\n';
          times.push_back(time);
          voltages.push_back((-1)*voltage); //invert back
        }

        Float_t average_voltage = accumulate( voltages.begin(), voltages.end(), 0.0)/voltages.size();




        TH1F *voltageHistogram_1 = new TH1F("voltHist","voltHist",1000,0,average_voltage);


            for(int j = 0; j < voltages.size(); j++) //traca
        		{

            voltageHistogram_1->Fill(voltages.at(j));
            }
            voltageHistogram_1->Fit("gaus");

            Float_t Vminimum = voltageHistogram_1->GetFunction("gaus")->GetParameter(1);
            Float_t min_sigma = voltageHistogram_1->GetFunction("gaus")->GetParameter(2);
              this->lowLevel = Vminimum;
              this->lowLevel_error = voltageHistogram_1->GetFunction("gaus")->GetParError(1) ;


            TH1F *voltageHistogram_2 = new TH1F("voltHist","voltHist",1000,average_voltage,*max_element(voltages.begin(), voltages.end()));

                for(int j = 0; j < voltages.size(); j++) //traca
            		{

                voltageHistogram_2->Fill(voltages.at(j));
                }
                voltageHistogram_2->Fit("gaus");

            Float_t Vmaximum = voltageHistogram_2->GetFunction("gaus")->GetParameter(1);
            Float_t max_sigma = voltageHistogram_2->GetFunction("gaus")->GetParameter(2);
            this->height = Vmaximum;
            this->height_error = voltageHistogram_2->GetFunction("gaus")->GetParError(1) ;

            /*
            for(int j = 0; j < voltages.size(); j++)
            {

              /*
            if(abs (voltages.at(j) - Vmaximum) < 2*max_sigma){
              voltages.at(j) = Vmaximum;
            }
            else if(abs (voltages.at(j) - Vminimum) < 2*min_sigma){
              voltages.at(j) = Vminimum;
            }
          }*/


          TGraph* gr;
          gr = new TGraph((Int_t)(times.size()),(times.data()),(voltages.data()));
          gr->SetTitle(";time [ns]; U [V]");
          //gr->GetYaxis()->SetTitleOffset(0.5);
            //voltageHistogram_1->Draw("AC*");
            gr->Draw("A*");
            //gr->GetXaxis()->SetLimits(0,10000);
/*------------            fil2 = fil;
            fil2.erase (fil2.end()-4, fil2.end());
            canvas_1->SaveAs("../simplePulses/" + fil2 +".png");*/

            /*
          TF1 *dataFit = new TF1("dataFit",myfunc,(Double_t)times.at(0),(Double_t)times.back());
          //dataFit->Draw("AC");
          TF1 *ConGauss = new TF1("ConGauss","[0]*TMath::Gaus(x,[1],[2])",(Double_t)times.at(0),(Double_t)times.back());
          //ConGauss->SetParameters(0.008,(Double_t)times.at(times.size()/2),4000);
          ConGauss->SetParameters(0.008,(Double_t)times.at(times.size()/2),400);
          //ConGauss->Draw("SAME");
           TF1Convolution *f_conv = new TF1Convolution("gaus","gaus",true);
           //f_conv->SetRange((Double_t)times.at(0),(Double_t)times.back());
           f_conv->SetRange(0,1);
           f_conv->SetNofPointsFFT(1000);
          TF1 *convoluted = new TF1("convoluted",*f_conv, /Double_t)times.at(0), (Double_t)times.back(), f_conv->GetNpar());
          convoluted->Draw("AC");

          //gr->Fit("convoluted");


          return;*/

          TLine *line = new TLine(0,average_voltage,40000,average_voltage);
            line->SetLineColor(kRed);
            line->Draw();

          TLine *lineMin = new TLine(0,Vminimum,40000,Vminimum);
            lineMin->SetLineColor(kRed);
            lineMin->Draw();

          TLine *lineMax = new TLine(0,Vmaximum,40000,Vmaximum);
            lineMax->SetLineColor(kRed);
            lineMax->Draw();

            for(int j = 0; j < voltages.size(); j++)
            {
              voltages.at(j) = voltages.at(j) - Vminimum;

            }

          canvas_1->Print((fil.erase(0,(string("/home/dannezlomnyj/Dropbox/Bakalarka/MeasuredData/SourceCalibration/").length()))+".png").c_str(),"png");
          //cout << (fil.erase(0,(string("/home/dannezlomnyj/Dropbox/Bakalarka/MeasuredData/SourceCalibration/").length()))).c_str() << endl;
          const Float_t Down_bound = 0.2* (Vmaximum-Vminimum);
          const Float_t Up_bound = 0.8 * (Vmaximum-Vminimum);

          cout << "down bound:" <<Down_bound << endl;
          cout << "up bound:" <<Up_bound << endl;

          //canvas_1->SaveAs("PWM.png");
          vector<vector<Float_t>> rising_times; //= new vector<int>;
          vector<vector<Float_t>>  rising_voltages;//= new vector<double>;
          vector<vector<Float_t>> falling_times; //= new vector<int>;
          vector<vector<Float_t>>  falling_voltages;//= new vector<double>;

          vector<Float_t> temp_times;
          vector<Float_t> temp_voltages;


          //Int_t d_no_pos, d_no_neg;
          //Float_t actual_voltage, actual_voltage_2, old_voltage = 0;
          //Int_t no_rises = 0;

          Int_t g_back = 0;
          Int_t g_up= 0;
          for (size_t i = 2; i < times.size() -6; i++)  { //from 2 and only to 6 before end
            //actual_voltage = voltages.at(i);
            g_back = 0;
            g_up = 0;
            if(((voltages.at(i-2) - (8 * voltages.at(i-1)) + (8 * voltages.at(i+1)) - voltages.at(i+2))/(24.0) > 0) && voltages.at(i+1) >= 0.5 *(Vmaximum-Vminimum) && voltages.at(i-1) < 0.5 *(Vmaximum-Vminimum)  ){

              cout << "new positive series" << endl;

              temp_times.clear();
              temp_voltages.clear();
              while(voltages.at(i-g_back) > Down_bound && ((i-g_back) > 0)){
                g_back++;
              }
              if((i-g_back) == 0){
                cout << "bounding problem" << endl;
                //g_back = 0;
                //g_up = 1;
                i += 1;
                continue;
              }
              if((i-g_back) > 4){
                  cout << "switching back" << endl;
                  g_back = g_back +4;
              }
              else{
                g_back = i;

              }
              /*else{
                g_back = g_back + (i-)
              }*/

              while(voltages.at(i-g_back+g_up) < Up_bound ){
                g_up++;
                cout << voltages.at(i-g_back+g_up) << endl;
                temp_voltages.push_back(voltages.at(i-g_back+g_up));
                temp_times.push_back(times.at(i-g_back+g_up));

              }
              for(int l = 1; l < 4; l++){
                temp_voltages.push_back(voltages.at(i-g_back+g_up+l));
                temp_times.push_back(times.at(i-g_back+g_up+l));

              }

              if((temp_times.back() - temp_times.at(0) > min_time && (temp_times.back() - temp_times.at(0) < max_time))){
                rising_times.push_back(temp_times);
                rising_voltages.push_back(temp_voltages);
              }




            }




            else if(((voltages.at(i-2) - (8 * voltages.at(i-1)) + (8 * voltages.at(i+1)) - voltages.at(i+2))/(24.0) < 0) && voltages.at(i+1) <= 0.5 *(Vmaximum-Vminimum) && voltages.at(i-1) > 0.5 *(Vmaximum-Vminimum)  ){

              cout << "new negative series" << endl;

              temp_times.clear();
              temp_voltages.clear();

              while(voltages.at(i-g_back) < Up_bound   && ((i-g_back) > 0)){
                g_back++;
              }
              if((i-g_back) == 0){
                cout << "bounding problem" << endl;
                //g_back = 0;
                //g_up = 1;
                i += 1;
                continue;
              }

              while((voltages.at(i-g_back+g_up) > Down_bound)){
                g_up++;
                cout << voltages.at(i-g_back+g_up) << endl;
                temp_voltages.push_back(voltages.at(i-g_back+g_up));
                temp_times.push_back(times.at(i-g_back+g_up));

              }
              if((temp_times.back() - temp_times.at(0) > min_time && (temp_times.back() - temp_times.at(0) < max_time))){
              falling_times.push_back(temp_times);
              falling_voltages.push_back(temp_voltages);
              }







            }

            i += (g_up - g_back);

            //----------------------------------------------------------------------old values -----------------------------------------------------------------------------------------------------------
            /*if(actual_voltage > Down_bound && actual_voltage < Up_bound){


                              //check for positive derivative
              d_no_pos = 0;
              //old_voltage = 0;
              if (actual_voltage < (0.5 *Vmaximum)) {


                actual_voltage_2 = actual_voltage;
                //old_voltage = actual_voltage_2;
                old_voltage = voltages.at(i-1);
                //while((actual_voltage_2 - old_voltage) > 0){
                while((voltages.at(i-2+d_no_pos) - (8 * voltages.at(i-1+d_no_pos)) + (8 * voltages.at(i+1+d_no_pos)) - voltages.at(i+2+d_no_pos))/(24.0) > min_slope){
                  if (!(actual_voltage_2 > Down_bound && actual_voltage_2 < Up_bound)){
                    d_no_pos = 0;
                    break;
                  }
                  old_voltage = actual_voltage_2;
                  d_no_pos++;
                  actual_voltage_2 = voltages.at(i+d_no_pos);
                  if(d_no_pos > no_deriv){
                    break;

                  }
                }
              }
              //old_voltage = 0;
                      //check for negative derivative
              d_no_neg = 0;
              if (actual_voltage > (0.5 *Vmaximum)) {
              actual_voltage_2 = actual_voltage;
              //old_voltage = actual_voltage_2;
              old_voltage = voltages.at(i-1);
              //while((actual_voltage_2 - old_voltage) < 0){
              while((voltages.at(i-2+d_no_neg) - (8 * voltages.at(i-1+d_no_neg)) + (8 * voltages.at(i+1+d_no_neg)) - voltages.at(i+2+d_no_neg))/(24.0) < -min_slope){
                if (!(actual_voltage_2 > Down_bound && actual_voltage_2 < Up_bound)){
                  d_no_neg = 0;
                  break;
                }
                old_voltage = actual_voltage_2;
                d_no_neg++;
                actual_voltage_2 = voltages.at(i+d_no_neg);
                if(d_no_neg > no_deriv){
                  break;

                }
              }
            }
              bool isEdge = true;
              Int_t j = 0;
              if(d_no_pos > no_deriv){

                cout << "new positive series" << endl;

                temp_times.clear();
                temp_voltages.clear();
                //actual_voltage_2 = 0;
                actual_voltage_2 = voltages.at(i);
                while(actual_voltage_2 < Up_bound){

                  temp_voltages.push_back(actual_voltage_2);
                  temp_times.push_back(times.at(i+j));
                  cout << actual_voltage_2 << endl;

                  j++;
                  actual_voltage_2 = voltages.at(i+j);
                  if(temp_times.back() - temp_times.at(0) > max_time_fall){
                    cout << "breaking, this is not a edge" << endl;
                    isEdge = false;
                    break;
                  }


                }
                if(isEdge){
                rising_times.push_back(temp_times);
                rising_voltages.push_back(temp_voltages);
              }
              }


              else if(d_no_neg > no_deriv){

                cout << "new negative series" << endl;

                temp_times.clear();
                temp_voltages.clear();
                //actual_voltage_2 = 0;*******
                actual_voltage_2 = voltages.at(i);


                while(actual_voltage_2 > Down_bound){

                temp_voltages.push_back(actual_voltage_2);
                temp_times.push_back(times.at(i+j));
                cout << actual_voltage_2 << endl;


                j++;
                actual_voltage_2 = voltages.at(i+j);
                if(temp_times.back() - temp_times.at(0) > max_time_rise){
                  cout << "breaking, this is not a edge" << endl;
                  isEdge = false;
                  break;
                }
                }
                if(isEdge){
                falling_times.push_back(temp_times);
                falling_voltages.push_back(temp_voltages);
              }

            }*/

              /*if((actual_voltage - old_voltage) > 0 && no_rises < 3){
              no_rises++;
              rising_times.push_back(times.at(i));
              rising_voltages.push_back(actual_voltage);
              }
              else if(no_rises < 3){
              no_rises = 0;

              }
              else if(no_rises >= 3){



              }

          }*/
          }


          //TF1 *edge_1 = new TF1("edge_1","0.0035*erf(0.025*(x-50)) + 0.0035",rising_times[0].at(0) - 500,rising_times[0].back()+500);
          TF1 *edge_1 = new TF1("edge_1","[0]*erf([1]*(x-[2])) + [3]",rising_times[0].at(0) - 20,rising_times[0].back()+20);
          TF1 *edge_2 = new TF1("edge_2","[0]*erf([1]*(x-[2])) + [3]",rising_times[1].at(0) -20,rising_times[1].back()+20);
          edge_1->SetParameters(0.0035,0.025,(rising_times[0].at(0) + rising_times[0].back())/2,0.0035);
          edge_2->SetParameters(0.0035,0.025,(rising_times[1].at(0) + rising_times[1].back())/2,0.0035);
          TGraph * funkce;
          /*TF1 *edge_1 = new TF1("edge_1","0.007*erf([0]*x) + [1]",rising_times[0].at(0) - 20,rising_times[0].back()+20);
          TF1 *edge_2 = new TF1("edge_2","0.007*erf([0]*x) + [1]",rising_times[1].at(0) -20,rising_times[1].back()+20);
          edge_1->SetParameters(0.003,0.001);
          edge_2->SetParameters(0.003,0.001);*/
          canvas_2 = new TCanvas("c2","fit1", 1400,900);
          TMultiGraph *mg = new TMultiGraph();
          //TF1 *f1 = new TF1("f1","[0]*x+[1])");
          mg->Add(gr);
           TGraph** gRise = (TGraph**) malloc(sizeof(TGraph*) * rising_voltages.size());
           TGraph** gFall = (TGraph**) malloc(sizeof(TGraph*) * falling_voltages.size());
           //gRise[0] = new TGraph((Int_t)(times.size()),(times.data()),(voltages.data()));
           cout << "here = ";
           Int_t k = 0;
          for (auto& itRow : rising_voltages)
            {   gRise[k] = new TGraph((Int_t)(rising_times[k].size()),(rising_times[k].data()),(itRow.data()));
                //gRise[k]->Fit("pol1");
                if(k == 0){
                gRise[k]->Fit("pol1");
                gRise[k]->SetMarkerColor(3);
                //cout << "a = " << gRise[k]->GetFunction("edge_1")->GetParameter(1) << " b = "<< gRise[k]->GetFunction("edge_1")->GetParameter(2) <<endl;
                mg->Add(gRise[k]);
                gRise[k]->Draw("A*");

                //funkce = new TGraph(edge_1,"");
                //funkce->SetMarkerColor(kRed);
                //mg->Add(funkce);
                //funkce->Draw("AC");
                //
                mg->Draw("ap");
                mg->GetXaxis()->SetLimits(0,200);
                canvas_2->Modified();
                canvas_2->Update();
                canvas_2->SaveAs("ErfFit.png");
                //return;
                //edge_1->SetParameters(0.0035,0.025,(rising_times[0].at(0) + rising_times[0].back())/2,0.0035);
                }
                else{
                  gRise[k]->Fit("pol1");
                  gRise[k]->SetMarkerColor(3);
                  cout << "a = " << gRise[k]->GetFunction("pol1")->GetParameter(0) << " b = "<< gRise[k]->GetFunction("pol1")->GetParameter(1) <<endl;
                  mg->Add(gRise[k]);
                  gRise[k]->Draw("A*");
                  canvas_2->SaveAs("ErfFit2.png");
                  //edge_2->SetParameters(0.0035,0.025,(rising_times[1].at(0) + rising_times[1].back())/2,0.0035);

                }

                std::cout << "cells = ";

                for (auto& itCell : itRow)
                {
                    std::cout << itCell << " ";
                }
                std::cout << std::endl;
                k++;
            }
            //return;

             /*TF1 *lin1 = new TF1("lin1","[0]*x+[1]",0,200);
             lin1->SetParameter(0,gRise[0]->GetFunction("pol1")->GetParameter(1));
             lin1->SetParameter(1,gRise[0]->GetFunction("pol1")->GetParameter(0));
             //TF1 *lin2 = new TF1("lin2","[0]*x+[1])",-3,3);
             TGraph * FuncG = new TGraph(lin1,"");*/
          //mg->Add(FuncG);

          mg->Draw("A*");
          mg->GetXaxis()->SetLimits(rising_times.at(0).at(0) -50,rising_times.at(0).back() +50);

          //c2->SaveAs("PWM2test.png");
          this->slope = (gRise[0]->GetFunction("pol1")->GetParameter(1) + gRise[1]->GetFunction("pol1")->GetParameter(1)) /2;
          this->slope_error = pow(((0.25)*pow(gRise[0]->GetFunction("pol1")->GetParError(1),2)) + ((0.25)*pow(gRise[1]->GetFunction("pol1")->GetParError(1),2)),0.5);
          this->rise_time = ((rising_times[0].back() - rising_times[0].at(0)) + (rising_times[1].back() - rising_times[1].at(0))) /2 ;
          this->rise_time_error = 0;
          this->drop_time = ((falling_times[0].back() - falling_times[0].at(0)) + (falling_times[1].back() - falling_times[1].at(0))) /2 ;
          //this->drop_time = falling_times[0].size() + falling_times[1].size();
          //this->drop_time_error = 0;
          //std::cout << "fall"<<rising_times[0].size() << endl;
          //  std::cout << rising_times[1].size() << endl;

          cout << "here" << '\n';
          //this->drop_time = ((falling_times[0].back() - falling_times[0].at(0)) + (falling_times[1].back() - falling_times[1].at(0))) /2 ;
        //  this->drop_time_error = 0;

          //canvas_2->Print("FirstRise.pdf");



          //canvas_3 = new TCanvas("c3","fit2", 600,600);
          mg->GetXaxis()->SetLimits(rising_times.at(1).at(0) -50,rising_times.at(1).back() +50);

          //canvas_2->Print("SecondRise.pdf");





          //fill the properties


          delete voltageHistogram_1;
          delete voltageHistogram_2;
          delete canvas_1;
          delete canvas_2;
          delete mg;
          delete line;
          delete lineMin;
          delete lineMax;


          /*for(int i=0; i<rising_voltages.size()-1; i++){
            free(gRise[i]);

          }
            free(gRise);
          for(int i=0; i<falling_voltages.size()-1; i++){
            free(gFall[i]);

          }
            free(gFall);*/









        }

    // ... more things...

    /*TCanvas Pulse:: *canvas_1;
    TCanvas Pulse:: *canvas_2;
    //TCanvas *canvas_3;
    Float_t Pulse:: height;
    Float_t Pulse:: height_error;
    Float_t Pulse:: lowLevel;
    Float_t Pulse:: lowLevel_error;
    Float_t Pulse:: rise_time;
    Float_t Pulse:: rise_time_error;
    //Float_t drop_time;
    //Float_t drop_time_error;
    Float_t Pulse:: slope;
    Float_t Pulse:: slope_error;*/
//};
