//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Timers;


   
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private static System.Timers.Timer timWord;
        bool timePassed = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            //Set timer
            timWord = new System.Timers.Timer(3000);
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            

                            this.Alfabet(jointPoints[JointType.HandLeft], jointPoints[JointType.HandRight], jointPoints[JointType.ShoulderLeft], jointPoints[JointType.ShoulderRight],jointPoints[JointType.ElbowLeft], jointPoints[JointType.ElbowRight]);
                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }
        

        private void Alfabet(Point HandLeftPosition, Point HandRightPosition, Point ShoulderLeftPosition, Point ShoulderRightPosition, Point ElbowLeftPosition, Point ElbowRightPosition)
        {
            int LeftArmPos = 0;
            int RightArmPos = 0;
            string Letter = "Def";
            double HLPX = HandLeftPosition.X;
            double HLPY = HandLeftPosition.Y;
            double HRPX = HandRightPosition.X;
            double HRPY = HandRightPosition.Y;

            double SLPX = ShoulderLeftPosition.X;
            double SLPY = ShoulderLeftPosition.Y;
            double SRPX = ShoulderRightPosition.X;
            double SRPY = ShoulderRightPosition.Y;

            double ELPX = ElbowLeftPosition.X;
            double ELPY = ElbowLeftPosition.Y;
            double ERPX = ElbowRightPosition.X;
            double ERPY = ElbowRightPosition.Y;

            double ABXValue;
            double ABYValue;
            double BCXValue;
            double BCYValue;
            double ACXValue;
            double ACYValue;
            double AB;
            double BC;
            double AC;

            double DEXValue;
            double DEYValue;
            double EFXValue;
            double EFYValue;
            double DFXValue;
            double DFYValue;
            double DE;
            double EF;
            double DF;

            ABXValue = Math.Abs(ELPX - HLPX);
            ABYValue = Math.Abs(ELPY - HLPY);
            BCXValue = Math.Abs(ELPX - SLPX);
            BCYValue = Math.Abs(ELPY - SLPY);
            ACXValue = Math.Abs(HLPX - SLPX);
            ACYValue = Math.Abs(HLPY - SLPY);

            DEXValue = Math.Abs(ERPX - HRPX);
            DEYValue = Math.Abs(ERPY - HRPY);
            EFXValue = Math.Abs(ERPX - SRPX);
            EFYValue = Math.Abs(ERPY - SRPY);
            DFXValue = Math.Abs(HRPX - SRPX);
            DFYValue = Math.Abs(HRPY - SRPY);

            //Calculations to see if the left arm is in a straight line
            AB = Math.Sqrt(Math.Pow(ABXValue, 2) + Math.Pow(ABYValue, 2));
            BC = Math.Sqrt(Math.Pow(BCXValue, 2) + Math.Pow(BCYValue, 2));
            AC = Math.Sqrt(Math.Pow(ACXValue, 2) + Math.Pow(ACYValue, 2));

            DE = Math.Sqrt(Math.Pow(DEXValue, 2) + Math.Pow(DEYValue, 2));
            EF = Math.Sqrt(Math.Pow(EFXValue, 2) + Math.Pow(EFYValue, 2));
            DF = Math.Sqrt(Math.Pow(DFXValue, 2) + Math.Pow(DFYValue, 2));

            if (AB + BC < AC + 4 && AB + BC > AC - 4)
            {
                if ((HLPX < SLPX + 30 && HLPX > SLPX - 30))
                {
                    if (HLPY < SLPY)
                    {
                        //Rigt arm pointing up 12h clock
                        LeftArmPos = 12;
                    }
                    else
                    {
                        //Right arm pointing down 6h clock
                        LeftArmPos = 6;
                    }

                }
                else
                {
                    if ((HLPY < SLPY + 30 && HLPY > SLPY - 30))
                    {
                        if (HLPX < SLPX)
                        {
                            //Right arm pointing left 9h clock
                            LeftArmPos = 9;
                        }
                        else
                        {
                            //Rigt arm pointing right 3h clock
                            LeftArmPos = 3;
                        }

                    }
                    else
                    {
                        if ((ACXValue < ACYValue + 20 && ACXValue > ACYValue - 20))
                        {
                            if (HLPY < SLPY && HLPX < SLPX)
                            {
                                //Rigt arm pointing left and a bit up 10:30h clock
                                LeftArmPos = 10;
                            }
                            else
                            {
                                if (HLPY > SLPY && HLPX < SLPX)
                                {
                                    //Right arm pointing left and a bit down 7:30h clock
                                    LeftArmPos = 7;
                                }
                                else
                                {
                                    if (HLPY < SLPY && HLPX > SLPX)
                                    {
                                        //right arm pointing right and a bit up 2:30h clock
                                        LeftArmPos = 1;
                                    }
                                    else
                                    {
                                        if (HLPY > SLPY && HLPX > SLPX)
                                        {
                                            //right arm pointing right and a bit down 4:30h clock
                                            LeftArmPos = 4;
                                        }
                                        else
                                        {
                                            LeftArmPos = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (DE + EF < DF + 4 && DE + EF > DF - 4)
            {
                if ((HRPX < SRPX + 30 && HRPX > SRPX - 30))
                {
                    if (HRPY < SRPY)
                    {
                        //Rigt arm pointing up 12h clock
                        RightArmPos = 12;
                    }
                    else
                    {
                        //Right arm pointing down 6h clock
                        RightArmPos = 6;
                    }

                }
                else
                {
                    if ((HRPY < SRPY + 30 && HRPY > SRPY - 30))
                    {
                        if (HRPX < SRPX)
                        {
                            //Right arm pointing left 9h clock
                            RightArmPos = 9;
                        }
                        else
                        {
                            //Rigt arm pointing right 3h clock
                            RightArmPos = 3;
                        }

                    }
                    else
                    {
                        if ((DFXValue < DFYValue + 20 && DFXValue > DFYValue - 20))
                        {
                            if (HRPY < SRPY && HRPX < SRPX)
                            {
                                //Rigt arm pointing left and a bit up 10:30h clock
                                RightArmPos = 10;
                            }
                            else
                            {
                                if (HRPY > SRPY && HRPX < SRPX)
                                {
                                    //Right arm pointing left and a bit down 7:30h clock
                                    RightArmPos = 7;
                                }
                                else
                                {
                                    if (HRPY < SRPY && HRPX > SRPX)
                                    {
                                        //right arm pointing right and a bit up 2:30h clock
                                        RightArmPos = 1;
                                    }
                                    else
                                    {
                                        if (HRPY > SRPY && HRPX > SRPX)
                                        {
                                            //right arm pointing right and a bit down 4:30h clock
                                            RightArmPos = 4;
                                        }
                                        else
                                        {
                                            RightArmPos = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            tbxrighthand.Text = Convert.ToString(RightArmPos);
            

            //Define all letters
            if(RightArmPos == 4)
            {
                if (LeftArmPos == 6)
                    Letter = "A";
                if (LeftArmPos == 12)
                    Letter = "K";
                if (LeftArmPos == 10)
                    Letter = "L";
                if (LeftArmPos == 9)
                    Letter = "M";
                if (LeftArmPos == 7)
                    Letter = "N";
            }
            else
            {
                if(RightArmPos == 3)
                {
                    if (LeftArmPos == 6)
                        Letter = "B";
                    if (LeftArmPos == 4)
                        Letter = "H";
                    if (LeftArmPos == 1)
                        Letter = "O";
                    if (LeftArmPos == 12)
                        Letter = "P";
                    if (LeftArmPos == 10)
                        Letter = "Q";
                    if (LeftArmPos == 9)
                        Letter = "R";
                    if (LeftArmPos == 7)
                        Letter = "S";
                }
                else
                {
                    if (RightArmPos == 1)
                    {
                        if (LeftArmPos == 6)
                            Letter = "C";
                        if (LeftArmPos == 12)
                            Letter = "T";
                        if (LeftArmPos == 10)
                            Letter = "U";
                        if (LeftArmPos == 9)
                            Letter = "Y";
                        if (LeftArmPos == 7)
                            Letter = "RESET";
                    }
                    else
                    {
                        if(RightArmPos == 12)
                        {
                            if (LeftArmPos == 6)
                                Letter = "D";
                            if (LeftArmPos == 4)
                                Letter = "I";
                            if (LeftArmPos == 9)
                                Letter = "J";
                            if (LeftArmPos == 7)
                                Letter = "V";
                        }
                        else
                        {
                            if(RightArmPos == 6)
                            {
                                if (LeftArmPos == 10)
                                    Letter = "E";
                                if (LeftArmPos == 9)
                                    Letter = "F";
                                if (LeftArmPos == 7)
                                    Letter = "G";
                                if (LeftArmPos == 6)
                                    Letter = "EOW SPACE";
                            }
                            else
                            {
                                if (RightArmPos == 10)
                                {
                                    if (LeftArmPos == 9)
                                        Letter = "W";
                                    if (LeftArmPos == 7)
                                        Letter = "X";
                                }
                                else
                                {
                                    if (RightArmPos == 7)
                                    {
                                        if (LeftArmPos == 7)
                                            Letter = "Z";
                                    }
                                }
                            }
                        }
                    }
                }
                    


            }

            tbxLetter.Text = Letter;

            MakeWord(Letter);
        }

        private void MakeWord(string Let)
        {
            bool LetterFailed = false;
            string word = "";
            string newWord;
            string LetterNow = "";
            if (Let != "Def")
            {

                //Set timer
                timWord = new System.Timers.Timer(3000);
                timWord.Elapsed += OnTimedEvent;
                timWord.Enabled = true;
                timWord.Start();
                if(LetterNow != Let && timePassed == false)
                {
                    LetterFailed = true;
                    timWord.Stop();
                }
                if (timePassed == true && LetterFailed == false)
                {
                    word = word + Let;
                    if(Let == "cancel")
                    {
                        word = "";
                    }
                    timePassed = false;
                    timWord.Stop();
                }
                LetterNow = Let;
                tbxlefthand.Text = word;
            }
        }

        private void OnTimedEvent(object sender, ElapsedEventArgs e)
        {
            timePassed = true;
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    
                    break;
                    
                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
