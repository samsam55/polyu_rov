//mate 2018

// Configuration.
const int DELAYLOOPTIME = 20;

// Pins' number arrays' size.
const int CAMSIZE = 8;
const int VALVESIZE = 2;
const int ARMOUTSIZE = 7;
const int ARMINSIZE = 2;

// Pins' number.
//                           Cam1 Cam2 Cam3 Cam4 Cam5 Cam6 Cam7 Cam8
const int CAM[ CAMSIZE ] = {   A1,  A3,  A6,  A0,  A2,  A5,  A4,  A7 };
//                               Gripper1 Cylinder Gripper2
const int VALVE[ VALVESIZE ] = {   32   ,      30 };
//                                             Arm1 UP DOWN   Arm2 UP DOWN   Arm3 UP DOWN Arm4 UP DOWN
const int ARM[ ARMOUTSIZE ][ ARMINSIZE ] = {  { 50,  52 }, { 42,  44 },   {38, 36},  { 46, 48  },   {28, 34},  {32, 30},  {34, 28}}; //{ 42,  44 }, { 50,  52 },  {36, 38},  { 48, 46  },   {28, 34},  {32, 30},  {34, 28}
// Bitstream memory.
//                      Cam1 Cam2 Cam3 Cam4 Cam5 Cam6 Cam7 Cam8
byte cam[ CAMSIZE ] = {    0,   0,   0,   0,   0,   0,   0,   0 }; // 0s' will be changed to [0,180].
//                          Cylinder Gripper1 Gripper2
byte valve[ VALVESIZE ] = {        0,       0 };          // 0s' will be changed to [0,1].
//                         Arm1 Arm2 Arm3 Arm4
byte arm[ ARMOUTSIZE ] = {    0,   0,   0 , 0};                   // arm1-4 0s' will be changed to [0,2].
const byte CAMUPDATEMARGIN = 5;
const int UPDATECAMIDLECYCLES = 5;
int updateCamIdle = UPDATECAMIDLECYCLES;
bool isUpdatedCam = false;
bool isUpdatedValve = false;
bool isUpdatedArm = false;

void setup()
{
  // Cam.
  for ( int camIdx = 0; camIdx < CAMSIZE; ++camIdx )
  {
    pinMode( CAM[ camIdx ], INPUT );
  }

  // Valve.
  for ( int valveIdx = 0; valveIdx < VALVESIZE; ++valveIdx )
  {
    pinMode( VALVE[ valveIdx ], INPUT );
  }

  // Arm.
  for ( int armOutIdx = 0; armOutIdx < ARMOUTSIZE; ++armOutIdx )
  {
    for ( int armInIdx = 0; armInIdx < ARMINSIZE; ++armInIdx )
    {
      pinMode( ARM[ armOutIdx ][ armInIdx ], INPUT );
    }
  }

  // Speed.
  Serial.begin( 115200 );
}

void loop()
{

  // Cam.
  isUpdatedCam = false;
  for ( int camIdx = 0; camIdx < CAMSIZE; ++camIdx )
  {
    int valInt = analogRead( CAM[ camIdx ] );
    valInt = map( valInt, 0, 1023, 180, 0 );
    if ( camIdx == 1 || camIdx == 5 || camIdx == 6)
      valInt = map( valInt, 180, 0, 0, 180 );
    byte val = ( byte ) ( valInt );
    if ( abs( cam[ camIdx ] - val ) >= CAMUPDATEMARGIN )
    {
      isUpdatedCam = true;
      cam[ camIdx ] = val;
    }
  }

  // Valve.
  isUpdatedValve = false;
  for ( int valveIdx = 0; valveIdx < VALVESIZE; ++valveIdx )
  {
    bool valBool = ( digitalRead( VALVE[ valveIdx ] ) == HIGH );
    byte val = ( valBool ? ( byte ) ( 1 ) : ( byte ) ( 0 ) );
    if ( valve[ valveIdx ] != val )
    {
      isUpdatedValve = true;
      valve[ valveIdx ] = val;
    }
  }

  // Arm.
  isUpdatedArm = false;
  for ( int armOutIdx = 0; armOutIdx < ARMOUTSIZE; ++armOutIdx )
  {
    bool armInVal[ ARMINSIZE ] = { false, false };
    for ( int armInIdx = 0; armInIdx < ARMINSIZE; ++armInIdx )
    {
      armInVal[ armInIdx ] = ( digitalRead( ARM[ armOutIdx ][ armInIdx ] ) == HIGH );
    }
    byte val = ( ( armInVal[ 0 ] ) ? ( byte ) ( 1 ) :
                 ( ( armInVal[ 1 ] ) ? ( byte ) ( 2 ) :
                   ( byte ) ( 0 ) ) );
    if ( arm[ armOutIdx ] != val )
    {
      isUpdatedArm = true;
      arm[ armOutIdx ] = val;
    }
  }

  // Serial write of cam.
  if (  isUpdatedCam ) //isUpdatedCam
  {
    updateCamIdle = UPDATECAMIDLECYCLES;
  }
  else
  {
    if ( updateCamIdle == 0 )
    {
      Serial.write( byte( 0xC1 ) );
      Serial.write( cam, CAMSIZE );
    }
    if ( updateCamIdle >= 0 )
    {
      --updateCamIdle;
    }
  }
  /*
    if ( isUpdatedValve )
    {
      // Serial write of valve.
      Serial.write( byte( 0xD1 ) );
      Serial.write( valve, VALVESIZE );
    }
  */
  if ( isUpdatedArm )
  {
    // Serial write of arm.
    Serial.write( byte( 0xD1 ) );
    Serial.write( arm, ARMOUTSIZE );
    //Serial.write( arm, 3 );
  }

  delay( DELAYLOOPTIME );

  /*
    if (isUpdatedArm || isUpdatedValve || isUpdatedCam) {
    Serial.println( "" );
    // Debug print of cam.
    Serial.print( "DEBUG >> cam:(0xC1)" );
    for ( int camIdx = 0; camIdx < CAMSIZE; ++camIdx )
    {
      Serial.print( "," );
      Serial.print( cam[ camIdx ] );
    }
    // Debug print of valve.
    Serial.println();
    Serial.print( " >> valve:(0xD1)" );
    for ( int valveIdx = 0; valveIdx < VALVESIZE; ++valveIdx )
    {
      Serial.print( "," );
      Serial.print( valve[ valveIdx ] );
    }
    // Debug print of arm.
    Serial.println();
    Serial.print( " >> arm:(0xD2)" );
    for ( int armIdx = 0; armIdx < ARMOUTSIZE; ++armIdx )
    {
      Serial.print( "," );
      Serial.print( arm[ armIdx ] );
    }
    Serial.println( "" );
    }
  */
}
