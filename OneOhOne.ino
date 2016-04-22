// OneOhOne.ino
#include <CurieIMU.h>
#include <BMI160.h>
#include <stdio.h>

class Worker
{
   friend class Program;

public:
   virtual void DoWork() {};
   virtual void Setup() {};

   bool IsLast() { return this->pNext == NULL; }

   Worker &operator+(Worker &refNew)
   {
      Worker *pLast = this;

      while (pLast->pNext)
         pLast = pLast->pNext;

      pLast->pNext = &refNew;

      return *this;
   }

   Worker &operator++()
   {
      return this->IsLast() ? *this : *(this->pNext);
   }

private:
   Worker * pNext = NULL;
};

class App101
{
public:
   App101()
   {
      instance = this;  // singleton
   }

   virtual void OnSetup()
   {
      Worker &refNext = this->headWorker;

      while (!refNext.IsLast)
      {
         refNext = refNext++;
         refNext.Setup();
      }
   }

   virtual void OnLoop()
   {
      Worker &refNext = this->headWorker;

      while (!refNext.IsLast)
      {
         refNext = refNext++;
         refNext.DoWork();
      }
   }

   static void Setup()
   {
      instance->OnSetup();
   }

   static void Loop()
   {
      instance->OnLoop();
   }

   Worker &operator+=(Worker &newWorker)
   {
      this->headWorker += newWorker;
      return this->headWorker;
   }

private:
   Worker  headWorker;  // empty worker - head of list

   static App101 *instance;
};

class Program
{
public:
   Program()
   {
   }

   Program(Worker &refWorker)
   {
      this->AddWorker(refWorker);
   }

   void Setup(bool serial = true)
   {
      this->serial = serial;
      if (this->serial)
         Serial.begin(this->serialBaudRate);

      Worker *pWorker = this->pWorkers;

      while (pWorker != NULL)
      {
         pWorker->Setup();
         pWorker = pWorker->pNext;
      }
   }

   void AddWorker(Worker &refWorker)
   {
      if (this->pWorkers == NULL)
         this->pWorkers = &refWorker;
      else
      {
         Worker *pLast = this->pWorkers;

         while (pLast && pLast->pNext)
            pLast = pLast->pNext;

         pLast->pNext = &refWorker;
      }
   }

   void Loop()
   {
      Worker *pWorker = this->pWorkers;

      while (pWorker != NULL)
      {
         pWorker->DoWork();
         pWorker = pWorker->pNext;
      }
   }

private:
   bool     serial = true;
   uint     serialBaudRate = 115200;
   Worker * pWorkers = NULL;
};

class IntervalWorker : public Worker
{
public:
   IntervalWorker(uint64_t msInterval)
   {
      this->interval = msInterval;
   }

   void Setup()
   {
      Worker::Setup();
      this->lastChange = millis();
   }

   void DoWork()
   {
      uint64_t ms = millis();

      Worker::DoWork();

      if ((ms - this->lastChange) > this->interval)
      {
         this->lastChange = ms;
         this->OnTimer(ms);
      }
   }

   virtual void OnTimer(uint64_t ms)
   {
   }

private:
   uint64_t lastChange;
   uint64_t interval;
};

class Blinker : public IntervalWorker
{
public:
   Blinker(uint64_t msInterval) : 
      IntervalWorker(msInterval)
   {
   }

   void Setup()
   {
      IntervalWorker::Setup();
      pinMode(LED_BUILTIN, OUTPUT);
   }

   void OnTimer(uint64_t ms)
   {
      digitalWrite(LED_BUILTIN, this->isOff ? HIGH : LOW);
      this->isOff = !this->isOff;
   }

private:
   bool     isOff = true;
};

class Gyro : public IntervalWorker
{
public:
   Gyro() :
      IntervalWorker(1000ul)
   {
   }

   void Setup()
   {
      if (CurieIMU.begin())
      {
         CurieIMU.setAccelerometerRange(4);
         Serial.println("IMU ready.");
      }
      else
         Serial.println("Error init IMU");
   }

   void OnTimer(uint64_t ms)
   {
      // CurieIMU.getRotation(&rotationX, &rotationY, &rotationZ);
      CurieIMU.getMotion6(&this->aX, &this->aY, &this->aZ, &this->gX, &this->gY, &this->gZ);
      Serial.print(this->aX);
      Serial.print(',');
      Serial.print(this->aY);
      Serial.print(',');
      Serial.print(this->aZ);
      Serial.print(',');
      Serial.print(this->gX);
      Serial.print(',');
      Serial.print(this->gY);
      Serial.print(',');
      Serial.println(this->gZ);
   }

private:
   int16_t aX;
   int16_t aY;
   int16_t aZ;
   int16_t gX;
   int16_t gY;
   int16_t gZ;
};

class Tapper : public Worker
{
public:

   void Setup()
   {
      CurieIMU.attachInterrupt(interruptCallback);
      CurieIMU.setDetectionThreshold(CURIE_IMU_TAP, 750);
      CurieIMU.setDetectionDuration(CURIE_IMU_DOUBLE_TAP, 250);
      CurieIMU.interrupts(CURIE_IMU_DOUBLE_TAP);
      pinMode(LED_BUILTIN, OUTPUT);
   }

private:
   static void interruptCallback()
   {
      int times = 0;
      int duration = 500;

      if (CurieIMU.getInterruptStatus(CURIE_IMU_DOUBLE_TAP)) 
      {
         if (CurieIMU.tapDetected(X_AXIS, NEGATIVE) || CurieIMU.tapDetected(X_AXIS, POSITIVE))
            times = 1;
         else if (CurieIMU.tapDetected(Y_AXIS, NEGATIVE) || CurieIMU.tapDetected(Y_AXIS, POSITIVE))
            times = 2;
         else if (CurieIMU.tapDetected(Z_AXIS, NEGATIVE) || CurieIMU.tapDetected(Z_AXIS, POSITIVE))
            times = 3;
      }
      else if (CurieIMU.getInterruptStatus(CURIE_IMU_TAP))
      {
         duration = 1000;
         if (CurieIMU.tapDetected(X_AXIS, NEGATIVE) || CurieIMU.tapDetected(X_AXIS, POSITIVE))
            times = 1;
         else if (CurieIMU.tapDetected(Y_AXIS, NEGATIVE) || CurieIMU.tapDetected(Y_AXIS, POSITIVE))
            times = 2;
         else if (CurieIMU.tapDetected(Z_AXIS, NEGATIVE) || CurieIMU.tapDetected(Z_AXIS, POSITIVE))
            times = 3;
      }

      for (int x = 0; x < times; x++)
      {
         digitalWrite(LED_BUILTIN, HIGH);
         delay(duration);
         digitalWrite(LED_BUILTIN, LOW);
         delay(duration);
      }
   }
};

Blinker  blinker(1000ul);
App101   App;
Gyro     gyro;

void setup() 
{
   App += blinker + gyro;
   App.Setup(); 
}

void loop() { App.Loop(); }

//Blinker blinker(1000ul);
//Gyro gyro;
//// Program app(blinker);
//Tapper tapper;
//Program app;
//
//void setup() 
//{
//   // put your setup code here, to run once:
//
//   app.AddWorker(gyro);
//   app.AddWorker(tapper);
//   app.Setup();
//
//   Serial.println("Ready!");
//}
//
//void loop() 
//{
//   // put your main code here, to run repeatedly:
//   app.Loop();
//   //digitalWrite(LED_BUILTIN, HIGH);
//   //delay(1500);
//   //digitalWrite(LED_BUILTIN, LOW);
//   //delay(1500);
//}
