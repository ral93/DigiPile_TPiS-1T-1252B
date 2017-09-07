/***************************************************************************************************************
Arduino-Code for DigiPile, Copyright (C) 2017 Ralph Heim 
incl. Calibration Process, Copyright (C) 2017 Excelitas Technologies GmbH & Co. KG

Author: Ralph Heim 
Last change: 22.08.2017 

Used documents: 
- Tamplate for Calibrationpart by EXCELITAS: "Technical Guide to a calibrated Lookup Table on a μC"
- Application Notes and Datasheet of DigiPile Family 

Hardware:
- Board: Arduino UNO with ATmega328P
- Sensor: DigiPile Thermopile Sensor TPiS 1T 1252B / 5058
 
Voltage devider with 5k und 10k between DirectLink and Arduino I/O 
100nF Capicator between VDD and GND 

This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

****************************************************************************************************************/



// constants and Calibration values
float nTemperatures = 20;
float Temperatures[20];
float Counts[20];
float k = 1.348e-6; // any value
float Tca = 293.15; // any value in K   
float w = 3.8; // power 4-δ

// Calibration values for T_ambient und U0
float T1_ptat = 24.09 + 273.15; // add the first value[K] measured by the reference thermometer at lower end of lookup-table
float U1_ptat = 7944; // add corresponding Tamb-Output
float T2_ptat = 273.15+39.7; // add second value[K] measured by the reference thermometer at upper end of lookup-table
float U2_ptat = 9379; // add corresponding Tamb-Output 
float U0 = 64520; // add corresponding Counts for the thermopile offset (corresponds to decimal value/counts(dcO) of T_object as soon as T_ambient = T_object)

float PTAT_offset_ADC = U1_ptat; 
float PTAT_offset_Temp = T1_ptat;
float PTAT_slope = (T2_ptat-T1_ptat)/(U2_ptat-U1_ptat);


// Calibration values for T_object
float T1_ptat_amb = 7988; // Add measured T_amb [decA; measurment in counts] during measurment of Black Body!
float T1 = 34.2+273.15 ; // Add the Temperture of the BB object measured by the Reference Thermometer 
float U1 = 67120; // Add the Temperture (in counts) of the BB object measured by the DigiPile

//***************************************************************************************************************

// Initialisation for Sensor-Comunication via DirectLink
boolean Tob[17]; //Tobject--> 17-bit
boolean Tamb[14];// Tambient--> 14-bit
unsigned long decO; // decimal value for Tob
unsigned long decA; // decimal value for Tamb

// Direct Link: Data Output/INPUT
const int DPDL = 2; 
//const int DLIN = 9;   
 
const unsigned long b = 1UL; // unsigned long for bit shifting 

// read every 10 ms read out rate < 15 ms)
const long intervall = 10;
const long intervall2 = 1000; // display every second  
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;

//***************************************************************************************************************

//'main()'
void setup() 
{
 Serial.begin(9600); // Initialize Serial 
 //Serial.println("Start");
 
 // value range for T_object: 23-43 °C. If T_object is not within this range, the 0°C/-273,15 K will be displayed
 for(int i=0;i<20;i++){
    double t = 0;
    t=i+23;
    Temperatures[i]= t; // 
  }


  // Calculation in Counts for T_object
  for(int i=0;i<20;i++){
     Counts[i] = U0 + (k * (pow(Temperatures[i]+273.15,w)-pow(Tca,w))); // // before calibration U0, k and Tca are predfined by any value
  }
  
  Calibrate();
  delay(5);
}


void loop() {
   
  unsigned long currentMillis = millis();
 
  if (currentMillis - previousMillis >= intervall){
 
  previousMillis = currentMillis;
  decO=0; 
  decA=0;
  
  Read();
  
  pinMode(DPDL,OUTPUT);
  digitalWrite(DPDL,LOW);
  //delayMicroseconds(0.2);
  pinMode(DPDL,INPUT_PULLUP);
  }

 if (currentMillis - previousMillis2 >= intervall2){
  previousMillis2 = currentMillis;
  Serial.print(currentMillis/1000);
  Serial.print("\t\t");
  PrintTo();
  Serial.print("\t\t");
  PrintTa();
  Serial.println();
  }
}



/***************************************************************************************************************
 * Methods for Calibration*************************************************************************************
***************************************************************************************************************/
 
// Calibration Calculation
// Look Up Table

//Interpolation for T_object values:
float Interpolate(float val, unsigned char invert) { 
  float val_high, val_low, result_high, result_low; 
  unsigned char ival; 
  ival = nTemperatures-2; 
  while(ival) {
    // Temperature --> Counts
   if (invert != 1){ 
    val_low = 273.15+Temperatures[ival]; 
    val_high = 273.15+Temperatures[ival+1]; 
    result_low = Counts[ival]; 
    result_high = Counts[ival+1]; 
    } 
    
    // Counts --> Temperature
  else { 
    val_low = Counts[ival]; 
    val_high = Counts[ival+1]; 
    result_low = 273.15+Temperatures[ival]; 
    result_high = 273.15+Temperatures[ival+1]; 
    }  

    
  if (val_low <= val && val < val_high) { 
    // weighted mean or linear interpolation 
    return (result_low + (val - val_low) / 
    (val_high - val_low) * (result_high - result_low)); 
    } 
    ival--; 
    } 
  return 0; 
}

//***************************************************************************************************************

// To calibrate T_obj and T_amb
void Calibrate(){ 
  unsigned char iTemperature; 
  float T1ambcount; 
  float T1count; 
  float T1amb; 

  // Calibrationpart T_amb
  PTAT_slope = (T2_ptat-T1_ptat)/(U2_ptat-U1_ptat); 
  PTAT_offset_ADC = U1_ptat; 
  PTAT_offset_Temp = T1_ptat;

   // Calibrationpart T_obj
  T1amb = GetTamb(T1_ptat_amb); 
  T1ambcount = Interpolate(T1amb, 0); 
  T1count = Interpolate(T1, 0); 
  iTemperature = nTemperatures; 
  // calibration of look up table
  while (iTemperature){ 
      Counts[iTemperature-1] = (Counts[iTemperature-1] - T1ambcount)* (U1 - U0) / (T1count - T1ambcount); 
      iTemperature--; 
      } 
}

//***************************************************************************************************************

// To get the claibrated value for T_ambient
float GetTamb(float val){
  return(PTAT_offset_Temp + (val - PTAT_offset_ADC) * PTAT_slope );
}

//***************************************************************************************************************

// To get the claibrated value for T_object:
float GetTobj(float valobj, float valamb){ 
   float valobj_corr; 
   valobj_corr = valobj - U0 + Interpolate(GetTamb(valamb), 0); // temperature compensation. Only possible with reference temperature
   return Interpolate(valobj_corr, 1); //in degree
 }


/***************************************************************************************************************
 * Methods for Comunication and Reading*************************************************************************
***************************************************************************************************************/

// to read data from direct link:

void Read(){

  // setup for reading
  pinMode(DPDL,OUTPUT);
  digitalWrite(DPDL,HIGH);
  delayMicroseconds(90); // setup time min 90µs
  
  //Reading Bits T_object
  for(int i=16;i>=0;i--){
    
    pinMode(DPDL, OUTPUT);
    digitalWrite(DPDL, LOW);
    //delayMicroseconds(10);

    pinMode(DPDL, OUTPUT);
    digitalWrite(DPDL,HIGH);
    //delayMicroseconds(10);
    
    pinMode(DPDL,INPUT_PULLUP);
    //pinMode(DLIN,INPUT_PULLUP);
    //pinMode(DPDL,INPUT);
  
   
    delayMicroseconds(5); // time wating to discharge for C = 10 pF for I/O 
     
     if(digitalRead(DPDL)==HIGH){
      Tob[i] = HIGH; //read
      decO+=b<<i;
    }

    else {
      Tob[i] = LOW; //read
      decO = decO;
    }

    pinMode(DPDL,INPUT); // set Port to high-input-impedance
 }

  //Reading Bits T_ambient
  for(int i=13;i>=0;i--){
    
    pinMode(DPDL, OUTPUT);
    digitalWrite(DPDL, LOW);
    //delayMicroseconds(10);

    pinMode(DPDL, OUTPUT);
    digitalWrite(DPDL,HIGH);
    //delayMicroseconds(10);
    
    pinMode(DPDL,INPUT_PULLUP);
    //pinMode(DLIN,INPUT_PULLUP);
    //pinMode(DPDL,INPUT);
    
    delayMicroseconds(5); // time wating to discharge for C = 10 pF at I/O-Pin or rather to reach HIGH or LOW Level

    if(digitalRead(DPDL)==HIGH){
      Tamb[i] = HIGH; //read
      decA+=b<<i;
    }

    else {
      Tamb[i] = digitalRead(DPDL); //read
      decA = decA;
    }
    pinMode(DPDL,INPUT); // set Port to high-input-impedance
  }
 return;
}

//***************************************************************************************************************

// to display the read data 
// ([16]/[13] => MSB)

void PrintTo (){
  for(int i=16;i>=0;i--){
    //Serial.print(Tob[i]);
  }
  //Serial.print("  ");
  
  Serial.print(decO);
  
  Serial.print("  ");
  Serial.print(GetTobj(decO,decA)-273.15);
}


void PrintTa (){
  for(int i=13;i>=0;i--){
    //Serial.print(Tamb[i]);
  }
  //Serial.print("  ");
  Serial.print(decA);
  Serial.print("  ");
  Serial.print((GetTamb(decA)-273.15));
}




