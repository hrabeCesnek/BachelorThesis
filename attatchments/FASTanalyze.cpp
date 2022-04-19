#include "FASTEventFile.h"
#include "FASTEvent.h"
#include "TFile.h"
#include "TH1.h"
#include "TF1.h"
#include "TGraphErrors.h"
#include "TMultiGraph.h"
#include "TTree.h"
#include "TMath.h"
#include <TCanvas.h>
#include <numeric>
#include <iostream>
#include <fstream>



#define REFERENCE_PIXEL 3
using namespace std;
using namespace TMath;
int errors = 0;
int event_no = 0;

double getMeanOfPixel(FASTPixel * pix){

		TH1F *pixelHist = new TH1F("pixelHist","pixelHist",300,300,5000);//500

		std::vector<double> tracy = pix->GetTrace();

		for(int j = 0; j < tracy.size(); j++) //traca
		{

                pixelHist->Fill((-1)*pix->GetCalibration()* tracy.at(j));
                }

		if(pixelHist->Integral() == 0){
		errors++;
		delete pixelHist;
		return -1;

		}
		cout << "sem" << endl;
		pixelHist->Fit("gaus");

		//singleWas->SaveAs(to_string(event_no) + ".png");
		double mean = pixelHist->GetFunction("gaus")->GetParameter(1);
		delete pixelHist;
		return mean;



}

int main(int argc, char *argv[]) {


		string side = argv[1];
    FASTEvent* myEvent = new FASTEvent();
    vector<double> *means = new vector<double> [4];
		TF1* f1 = new TF1("f1", "gaus", 1000, 2000);
		TF1* f2 = new TF1("f2", "gaus", 3000, 4500);
		TF1* f3 = new TF1("f3", "gaus", 500, 1200);
		string Chfunc = "f1";

				TH1F *hpx   = new TH1F [4]{
					TH1F("histogram 0","pixel 0; Signal Integral [N_{pe}];N_{events}",550,2500,150000),//250
					TH1F("histogram 1","pixel 1; Signal Integral [N_{pe}];N_{events}",550,2500,150000),
					TH1F("histogram 2","pixel 2; Signal Integral [N_{pe}];N_{events}",550,2500,150000),
					TH1F("histogram 3","pixel 3; Signal Integral [N_{pe}];N_{events}",550,2500,150000)
				    };


				for (int iFile = 0; iFile < argc; ++iFile)
		    {

					cout << argv[iFile] << endl;
				}




		TCanvas* singleWas = new TCanvas("singleWas","",800,600);
		
    for (int iFile = 2; iFile < argc; ++iFile)
    {
       
        FASTEventFile myFile(argv[iFile], FASTEventFile::eRead);
        myFile.SetBuffers(myEvent);
        cout << "reading file " << myFile.GetFileName() << " with " << myFile.GetNEvents() << " events." << endl;

        for (int iEvent = 0; iEvent < myFile.GetNEvents(); ++iEvent)
        {

            if (myFile.ReadEvent(myEvent) != FASTEventFile::eSuccess)
                continue;


            std::vector<FASTPixel> pixels = myEvent->GetPixels();

            for (int i = 0; i < pixels.size(); i++)
            {
		FASTPixel pixel1 = pixels.at(i);
		

		std::vector<double> tracy = pixel1.GetTrace();
		std::vector<double> tracyRaw;
		std::vector<double> times;
		double val;
		for(int j = 0; j < tracy.size(); j++) //traca
		{
		val = /*(-1)*pixel1.GetCalibration()*/ tracy.at(j);
		if(val > 15){
		//times.push_back(j*0.00000002);
		times.push_back(j*0.00000002);
		tracyRaw.push_back(val);
		//cout << val << endl;
		//pixelHist->Fill((-1)*pixel1.GetCalibration()* tracy.at(j));
		}
		}
		

		singleWas->Clear();
		TGraph* gr= new TGraph((Int_t)(times.size()),(times.data()),(tracyRaw.data()));
		TMultiGraph *mg = new TMultiGraph();
		gr->SetTitle("; t [s]; I [bl]");
	


		double pixel_val = 0;
		for(int j = 1; j < tracyRaw.size(); j++) //traca
		{
			//pixel_val = pixel_val + (0.00000002*((tracyRaw.at(j-1) + tracyRaw.at(j))/2));
			pixel_val = pixel_val + (((tracyRaw.at(j-1) + tracyRaw.at(j))/2));


		}


		//double pixel_val = gr->Integral(0,-1);




    //double pixel_val = std::accumulate(tracyRaw.begin(), tracyRaw.end(),0.0) ;
		//pixel_val = pixel_val/ (double)(tracyRaw.size());
		gr->SetMarkerColor(4);
		//gr->Fit("exp1","","",0.00004,0.0000475);
		gr->SetMarkerStyle(21);
		gr->Draw("AL");
		//pixelHist->Draw();
		mg->Add(gr);
		mg->Draw("AL");
		mg->GetXaxis()->SetLimits(0,0.00009);
		mg->SetMaximum(3000.);
		mg->SetMinimum(-100.);
		//mg->GetYaxis()->SetLimits(0,300);
		singleWas->Modified();
    singleWas->Update();
		//singleWas->Print("./singleEvents/out.pdf");
		tracyRaw.clear();
		times.clear();






		//double pixel_mean = getMeanOfPixel(&pixel1);
		if(pixel_val > 0){
		means[i].push_back(pixel_val);
		cout << "EvMean: " << means[i].back() << endl;
		}


	}
	/*if(iEvent > 100){
		singleWas->Print("./singleEvents/out.pdf]");
		return 0;



	}*/


				}

    }

	double final_mean [4]; //ze vzorce
	double standart_deviation [4];
	double final_mean_2 [4]; // z fitu
	double standart_deviation_2 [4];

	for(int pixel_id = 0; pixel_id < 4; pixel_id++){
		double sum = 0;
	    	for(int i = 0; i < means[pixel_id].size(); i++){
			sum = sum + means[pixel_id].at(i);


    		}

		final_mean [pixel_id] = sum/means[pixel_id].size();
		sum = 0;

		for(int i = 0; i < means[pixel_id].size(); i++){
			sum = sum + (pow(means[pixel_id].at(i) - final_mean[pixel_id],2));


    		}
		standart_deviation [pixel_id] = pow(sum/((means[pixel_id].size()-1)*means[pixel_id].size()),0.5);

		for(int i = 0; i < means[pixel_id].size(); i++){
		hpx[pixel_id].Fill(means[pixel_id].at(i));
		}
		
		hpx[pixel_id].Fit("gaus");
		cout << "here"  << endl;
		final_mean_2 [pixel_id] = /*hpx[pixel_id].GetMean();*/hpx[pixel_id].GetFunction("gaus")->GetParameter(1);
		standart_deviation_2 [pixel_id] = hpx[pixel_id].GetFunction("gaus")->GetParameter(2)/(sqrt(means[pixel_id].size()));//hpx[pixel_id].GetFunction("gaus")->GetParError(1);

	}

	for(int pixel_id = 0; pixel_id < 4; pixel_id++){
	cout << "pixel " << pixel_id << ":\n" << "STAT: x = " <<  final_mean [pixel_id] << " +- " << standart_deviation [pixel_id] << endl;
	cout << "FIT: x = " << final_mean_2 [pixel_id] << " +- " << standart_deviation_2 [pixel_id] << endl;
	}

	double calibration_constants [4];
	double calibration_deviations [4];

int refpix = LocMax(4, final_mean_2);


	calibration_constants [refpix] = 1;
	calibration_deviations [refpix] = 0;


	//double maxPix = final_mean_2[LocMax(4, final_mean_2)];


	for(int i = 0; i < 4; i++){
		if(i != refpix){
			calibration_constants [i] = final_mean_2 [i]/final_mean_2 [refpix];
			//calibration_deviations [i] = sqrt(pow((abs(final_mean [i]/(pow(final_mean [REFERENCE_PIXEL],2))) * standart_deviation [REFERENCE_PIXEL]),2) + pow((abs(1/final_mean [REFERENCE_PIXEL]) * standart_deviation[i]),2));
			calibration_deviations [i] = calibration_constants [i] * sqrt(pow((standart_deviation_2[i]/final_mean_2[i]),2) + pow((standart_deviation_2[refpix]/final_mean_2[refpix]),2));
		}
	}
    TCanvas* c1 = new TCanvas("c1","Title",800,600);
    c1->Divide(2,2);
    ofstream vysledek;
    char meno [25];
    char meno_2 [25];
    cout << "jmeno výsledného souboru:" << endl;
    scanf("%s",&meno);
    strcpy(meno_2,meno);
    strcat(meno,".csv");
    vysledek.open (meno);
    vysledek << "pixel_No,mean,error of mean, calib, error of calib\n";

    for(int i = 0; i < 4; i++){
    c1->cd(i+1);
		hpx[i].SetStats(0);
		hpx[i].GetXaxis()->SetLabelFont(53); //font is in pixel (see TAttText) myHist->GetXaxis->SetLabelSize(12)
		hpx[i].GetXaxis()-> SetTitleSize (0.045);
		hpx[i].GetXaxis()-> SetLabelSize (12);

		hpx[i].GetYaxis()->SetLabelFont(53); //font is in pixel (see TAttText) myHist->GetXaxis->SetLabelSize(12)
		hpx[i].GetYaxis()->SetTitleSize(0.045);
		hpx[i].GetYaxis()-> SetLabelSize (12);
    hpx[i].Draw();

    vysledek << i << ","<< final_mean_2[i] << "," << standart_deviation_2[i] << "," <<  calibration_constants [i] << "," << calibration_deviations [i] << ",\n";
    }

    vysledek.close();
    delete [] means;

    delete myEvent;
    strcat(meno_2,".pdf");
    c1->SaveAs(meno_2);

    delete [] hpx;
    cout << "chyby: " << errors << endl;
    return 0;
}
