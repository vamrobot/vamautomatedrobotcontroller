// VAM Automatic Robot Controller v1.0

using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using SimpleJSON;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Linq;
using System.IO;
using System.Diagnostics;
using System.IO.Ports;
using Technie.PhysicsCreator;

namespace vamrobotics
{
    class vamtest1 : MVRScript
    {
        protected JSONStorableStringChooser femaleChooser;
        protected JSONStorableStringChooser maleChooser;
        protected JSONStorableStringChooser serialPortChooser;
        protected JSONStorableStringChooser protocolChooser;
        protected JSONStorableStringChooser diagnosticWindowChooser;
        protected JSONStorableStringChooser baudChooser;
        protected JSONStorableString diagnostics;
        protected UIDynamicTextField diagnosticsTextField;
        protected UIDynamicButton connectToRobot;
        protected UIDynamicButton disconnectFromRobot;
        private List<string> playerList;
        private List<Female> females;
        private List<Male> males;
        private string[] comPortNames;

        // Output diagnostics to window
        private bool diagnosticsTextFieldOutput = false;

        // Serial port
        SerialPort serial;

        public override void Init()
        {
            try
            {
                pluginLabelJSON.val = "VAM Automatic Robot Controller v1.0";

                // Find all 'Person' Atoms currently in the scene
                Atom tempAtom;
                playerList = new List<string>();
                females = new List<Female>();
                males = new List<Male>();
                foreach (string atomUID in SuperController.singleton.GetAtomUIDs())
                {
                    tempAtom = SuperController.singleton.GetAtomByUid(atomUID);
                    if (tempAtom.type == "Person")
                    {
                        // Add new Player/'Person' Atom to playerList
                        playerList.Add(atomUID);

                        string sex = "female";

                        // Check if the penis tip collider exists to determine if male or female
                        try
                        {
                            //tempAtom.gameObject.GetComponentsInChildren<Collider>().First(s => s.gameObject.name == "AutoColliderGen3bHard");

                            //sex = "male";

                            foreach (Collider c in tempAtom.gameObject.GetComponentsInChildren<Collider>())
                            {
                                // Check if the penis tip collider exists to determine if male or female
                                if (c.gameObject.name == "AutoColliderGen3bHard")
                                {
                                    sex = "male";
                                }
                            }
                        }
                        catch
                        {
                            sex = "female";
                        }

                        if (sex == "female")
                        {
                            females.Add(new Female(atomUID));
                        }
                        else
                        {
                            males.Add(new Male(atomUID));
                        }
                    }
                }

                // Setup female selector
                List<string> femalesList = new List<string>();
                for (int f = 0; f < females.Count(); f++)
                {
                    femalesList.Add(females[f].name);
                }
                femaleChooser = new JSONStorableStringChooser("Female Chooser", femalesList, femalesList[0], "Select Female", FemaleChooserCallback);
                RegisterStringChooser(femaleChooser);
                CreatePopup(femaleChooser).labelWidth = 300f;

                // Setup male selector
                List<string> malesList = new List<string>();
                for (int m = 0; m < males.Count(); m++)
                {
                    malesList.Add(males[m].name);
                }
                maleChooser = new JSONStorableStringChooser("Male Chooser", malesList, malesList[0], "Select Male", MaleChooserCallback);
                RegisterStringChooser(maleChooser);
                CreatePopup(maleChooser).labelWidth = 300f;

                // Select Robot Output Protocol
                List<string> robotProtocol = new List<string>();
                robotProtocol.Add("T-Code v0.2");
                robotProtocol.Add("RSM");
                protocolChooser = new JSONStorableStringChooser("Output Mode Chooser", robotProtocol, "T-Code v0.2", "Select Robot Protocol");
                CreatePopup(protocolChooser, true).labelWidth = 300f;

                // Select COM Port
                comPortNames = System.IO.Ports.SerialPort.GetPortNames();
                List<string> comPorts = new List<string>(comPortNames);
                serialPortChooser = new JSONStorableStringChooser("COM Port Chooser", comPorts, "None", "Select COM port", SerialPortChooserCallback);
                CreatePopup(serialPortChooser, true).labelWidth = 300f;

                // Select Baud Rate
                List<string> baudRates = new List<string>();
                baudRates.Add("9600");
                baudRates.Add("19200");
                baudRates.Add("38400");
                baudRates.Add("74880");
                baudRates.Add("115200");
                baudRates.Add("230400");
                baudRates.Add("250000");
                baudChooser = new JSONStorableStringChooser("Baud Rate Chooser", baudRates, "115200", "Select Baud Rate");
                CreatePopup(baudChooser, true).labelWidth = 300f;

                // Setup connect to server button
                connectToRobot = CreateButton("Connect to Robot", true);
                connectToRobot.button.onClick.AddListener(ConnectToRobotCallback);

                // Setup disconnect from server button
                disconnectFromRobot = CreateButton("Disconnect from Robot", true);
                disconnectFromRobot.button.onClick.AddListener(DisconnectFromRobotCallback);

                CreateSpacer(true);

                // Setup diagnostic window selector
                List<string> diagnosticWindow = new List<string>();
                diagnosticWindow.Add("Hide");
                diagnosticWindow.Add("Show");
                diagnosticWindowChooser = new JSONStorableStringChooser("Console Show/Hide Chooser", diagnosticWindow, diagnosticWindow[0], "Diagnostics Window", DiagnosticWindowChooserCallback);
                RegisterStringChooser(diagnosticWindowChooser);
                CreatePopup(diagnosticWindowChooser, true).labelWidth = 300f;
            }
            catch (Exception e)
            {
                SuperController.LogError("Exception caught: " + e);
            }
        }

        protected void FixedUpdate()
        {
            // Update female(s) data
            for (int f = 0; f < females.Count(); f++)
            {
                females[f].update();
            }

            // Update male(s) data
            for (int m = 0; m < males.Count(); m++)
            {
                males[m].update();
            }

            // Cycle through any male to identify the male (if more than one male is in the scene) that was selected through
            // the VAM UI selector and who's penis is the basis for driving the robot
            for (int m = 0; m < males.Count(); m++)
            {
                if (males[m].name == maleChooser.val)
                {
                    // Once the male is found, cycle through any females in the scene
                    for (int f = 0; f < females.Count(); f++)
                    {
                        // Loop through the vagina triggers and look for any collisions between them and any other colliders in the scene
                        for (int i = 0; i < females[f].vaginaTriggers.Length; i++)
                        {
                            Dictionary<Collider, bool> vaginaDictionary = females[f].vaginaRigidbodies[i].GetComponent<CollisionTriggerEventHandler>().collidingWithDictionary;

                            // Check for any collisions, zero if no collisions currently occurring
                            if (vaginaDictionary.Count > 0)
                            {
                                foreach (KeyValuePair<Collider, bool> entry in vaginaDictionary)
                                {
                                    // Loop through penis colliders
                                    for (int c = 0; c < males[m].penisNames.Length; c++)
                                    {
                                        // Check if one of the penis colliders is having a collision with the current female's vagina trigger
                                        if (entry.Key.gameObject.name == males[m].penisNames[c])
                                        {
                                            // Set vagina insertion flag for consistency checking
                                            females[f].vaginaInsertionFlags[i] = true;

                                            if (c == (males[m].penisNames.Length - 1))
                                            {
                                                females[f].vaginaTipInsertionFlags[i] = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Consistency checking to ensure the penis is actually properly into the vagina
                        // Sometimes in the VAM environment the various rigidbodies and colliders can move through each other
                        // And this consistency checking avoids such spurious collisions from driving the robot in an uncontrolled manner
                        bool penisInserted = false;
                        int penisTipInsertionIndex = 0;

                        for (int i = 0; i < females[f].vaginaTriggers.Length; i++)
                        {
                            if (females[f].vaginaInsertionFlags[i])
                            {
                                for (int j = i; j >= 0; j--)
                                {
                                    if (females[f].vaginaInsertionFlags[j])
                                    {
                                        penisInserted = true;

                                        penisTipInsertionIndex = i;
                                    }
                                }
                            }
                            else
                            {
                                break;
                            }
                        }

                        // If the male's penis's tip collider wasn't triggering the furthest trigger in the female's vagina then the male's penis isn't inserted
                        if (!females[f].vaginaTipInsertionFlags[penisTipInsertionIndex])
                        {
                            penisInserted = false;
                        }

                        // Reset the vagina insertion and tip insertion flags
                        for (int i = 0; i < females[f].vaginaTriggers.Length; i++)
                        {
                            females[f].vaginaInsertionFlags[i] = false;

                            females[f].vaginaTipInsertionFlags[i] = false;
                        }

                        // If the penis is inserted into the vagina
                        if (penisInserted)
                        {
                            // Setup T-code reference coordinate system
                            // X(L0) is up/down in reference to the selected male's penis vector and is positive up
                            // Y(L1) is toward/away orthogonal to the selected male's penis vector and is positive away
                            // Z(L2) is left/right orthogonal to the selected male's penis vector and is positive left

                            // Vector from the selected male's penis's base to tip colliders
                            Vector3 refAxisX = males[m].penisVector;

                            // Use the vector from male's abdomenControl Rigidbody to the male's penis's base collider to establish the Z reference axis
                            Vector3 refAxisZ = Vector3.Cross(refAxisX, males[m].abdomen - males[m].penis[0]);
                            refAxisZ = (refAxisX.magnitude / refAxisZ.magnitude) * refAxisZ;

                            // Use the reference X and Z axes to establish the orthogonal Y axis
                            Vector3 refAxisY = Vector3.Cross(refAxisX, refAxisZ);
                            refAxisY = (refAxisX.magnitude / refAxisY.magnitude) * refAxisY;

                            // Vector from the female's vagina's labia trigger to the male's penis's base collider
                            Vector3 vaginaLabiaToPenisBase = females[f].vagina[0] - males[m].penis[0];

                            // Calculate X(L0) for robot based on the reference X axis and the vector from the female's vagina's labia trigger to the male's penis's base collider
                            float robotX = Vector3.Dot(refAxisX, vaginaLabiaToPenisBase) / (refAxisX.magnitude * refAxisX.magnitude);

                            // Calculate Y(L1) for robot based on the reference Y axis and the vector from the female's vagina's labia trigger to the male's penis's base collider
                            float robotY = 0.5f + Vector3.Dot(refAxisY, vaginaLabiaToPenisBase) / (refAxisX.magnitude * refAxisX.magnitude);

                            // Calculate Z(L2) for robot based on the reference Z axis and the vector from the female's vagina's labia trigger to the male's penis's base collider
                            float robotZ = 0.5f + Vector3.Dot(refAxisZ, vaginaLabiaToPenisBase) / (refAxisX.magnitude * refAxisX.magnitude);

                            // Vector from the female's vagina's labia to vagina triggers
                            Vector3 vaginaLabiaToVaginaTrigger = females[f].vagina[0] - females[f].vagina[1];

                            // Calculate RY(R1) for robot based on the reference Z axis and the vector from the female's vagina's labia to vagina triggers
                            float robotRYAngle = 90.0f - Vector3.Angle(refAxisZ, vaginaLabiaToVaginaTrigger);
                            float robotRY = 0.5f + robotRYAngle / 180.0f;

                            // Calculate RZ(R2) for robot based on the reference Y axis and the vector from the female's vagina's labia to vagina triggers
                            float robotRZAngle = -(90.0f - Vector3.Angle(refAxisY, vaginaLabiaToVaginaTrigger));
                            float robotRZ = 0.5f + robotRZAngle / 180.0f;

                            if (diagnosticsTextFieldOutput)
                            {
                                diagnostics.val = "Robot X(L0): " + robotX + "\n";
                                diagnostics.val += "Robot Y(L1): " + robotY + "\n";
                                diagnostics.val += "Robot Z(L2): " + robotZ + "\n";
                                diagnostics.val += "Robot RY(R1): " + robotRY + "\n";
                                diagnostics.val += "Robot RZ(R2): " + robotRZ + "\n";
                                diagnostics.val += "Robot RY(R1) Angle: " + robotRYAngle + "\n";
                                diagnostics.val += "Robot RZ(R2) Angle: " + robotRZAngle;
                            }

                            if (serial != null && serial.IsOpen)
                            {
                                SendCommandToRobot(robotX, robotY, robotZ, robotRY, robotRZ);
                            }
                        }

                        // Loop through the mouth triggers and look for any collisions between them and any other colliders in the scene
                        for (int i = 0; i < females[f].mouthTriggers.Length; i++)
                        {
                            Dictionary<Collider, bool> mouthDictionary = females[f].mouthRigidbodies[i].GetComponent<CollisionTriggerEventHandler>().collidingWithDictionary;

                            // Check for any collisions, zero if no collisions currently occurring
                            if (mouthDictionary.Count > 0)
                            {
                                foreach (KeyValuePair<Collider, bool> entry in mouthDictionary)
                                {
                                    // Loop through penis colliders
                                    for (int c = 0; c < males[m].penisNames.Length; c++)
                                    {
                                        // Check if one of the penis colliders is having a collision with the current female's mouth trigger
                                        if (entry.Key.gameObject.name == males[m].penisNames[c])
                                        {
                                            // Set mouth insertion flag for consistency checking
                                            females[f].mouthInsertionFlags[i] = true;

                                            if (c == (males[m].penisNames.Length - 1))
                                            {
                                                females[f].mouthTipInsertionFlags[i] = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Consistency checking to ensure the penis is actually properly into the mouth
                        // Sometimes in the VAM environment the various rigidbodies and colliders can move through each other
                        // And this consistency checking avoids such spurious collisions from driving the robot in an uncontrolled manner
                        penisInserted = false;
                        penisTipInsertionIndex = 0;

                        for (int i = 0; i < females[f].mouthTriggers.Length; i++)
                        {
                            if (females[f].mouthInsertionFlags[i])
                            {
                                for (int j = i; j >= 0; j--)
                                {
                                    if (females[f].mouthInsertionFlags[j])
                                    {
                                        penisInserted = true;

                                        penisTipInsertionIndex = i;
                                    }
                                }
                            }
                            else
                            {
                                break;
                            }
                        }

                        // If the male's penis's tip collider wasn't triggering the furthest trigger in the female's mouth then the male's penis isn't inserted
                        if (!females[f].mouthTipInsertionFlags[penisTipInsertionIndex])
                        {
                            penisInserted = false;
                        }

                        // Reset the mouth insertion  and tip insertion flags
                        for (int i = 0; i < females[f].mouthTriggers.Length; i++)
                        {
                            females[f].mouthInsertionFlags[i] = false;

                            females[f].mouthTipInsertionFlags[i] = false;
                        }

                        // If the penis is inserted into the mouth
                        if (penisInserted)
                        {
                            // Setup T-code reference coordinate system
                            // X(L0) is up/down in reference to the selected male's penis vector and is positive up
                            // Y(L1) is toward/away orthogonal to the selected male's penis vector and is positive away
                            // Z(L2) is left/right orthogonal to the selected male's penis vector and is positive left

                            // Vector from the selected male's penis's base to tip colliders
                            Vector3 refAxisX = males[m].penisVector;

                            // Use the vector from male's abdomenControl Rigidbody to the male's penis's base collider to establish the Z reference axis
                            Vector3 refAxisZ = Vector3.Cross(refAxisX, males[m].abdomen - males[m].penis[0]);
                            refAxisZ = (refAxisX.magnitude / refAxisZ.magnitude) * refAxisZ;

                            // Use the reference X and Z axes to establish the orthogonal Y axis
                            Vector3 refAxisY = Vector3.Cross(refAxisX, refAxisZ);
                            refAxisY = (refAxisX.magnitude / refAxisY.magnitude) * refAxisY;

                            // Vector from the female's mouth's lip trigger to the male's penis's base collider
                            Vector3 mouthLipToPenisBase = females[f].mouth[0] - males[m].penis[0];

                            // Calculate X(L0) for robot based on the reference X axis and the vector from the female's mouth's lip trigger to the male's penis's base collider
                            float robotX = Vector3.Dot(refAxisX, mouthLipToPenisBase) / (refAxisX.magnitude * refAxisX.magnitude);

                            // Calculate Y(L1) for robot based on the reference Y axis and the vector from the female's mouth's lip trigger to the male's penis's base collider
                            float robotY = 0.5f + Vector3.Dot(refAxisY, mouthLipToPenisBase) / (refAxisX.magnitude * refAxisX.magnitude);

                            // Calculate Z(L2) for robot based on the reference Z axis and the vector from the female's mouth's lip trigger to the male's penis's base collider
                            float robotZ = 0.5f + Vector3.Dot(refAxisZ, mouthLipToPenisBase) / (refAxisX.magnitude * refAxisX.magnitude);

                            // Vector from the female's mouth's lip to mouth triggers
                            Vector3 mouthLipToMouthTrigger = females[f].mouth[0] - females[f].mouth[1];

                            // Calculate RY(R1) for robot based on the reference Z axis and the vector from the female's mouth's lip to mouth triggers
                            float robotRYAngle = 90.0f - Vector3.Angle(refAxisZ, mouthLipToMouthTrigger);
                            float robotRY = 0.5f + robotRYAngle / 180.0f;

                            // Calculate RZ(R2) for robot based on the reference Y axis and the vector from the female's mouth's lip to mouth triggers
                            float robotRZAngle = -(90.0f - Vector3.Angle(refAxisY, mouthLipToMouthTrigger));
                            float robotRZ = 0.5f + robotRZAngle / 180.0f;

                            if (diagnosticsTextFieldOutput)
                            {
                                diagnostics.val = "Robot X(L0): " + robotX + "\n";
                                diagnostics.val += "Robot Y(L1): " + robotY + "\n";
                                diagnostics.val += "Robot Z(L2): " + robotZ + "\n";
                                diagnostics.val += "Robot RY(R1): " + robotRY + "\n";
                                diagnostics.val += "Robot RZ(R2): " + robotRZ + "\n";
                                diagnostics.val += "Robot RY(R1) Angle: " + robotRYAngle + "\n";
                                diagnostics.val += "Robot RZ(R2) Angle: " + robotRZAngle;
                            }

                            if (serial != null && serial.IsOpen)
                            {
                                SendCommandToRobot(robotX, robotY, robotZ, robotRY, robotRZ);
                            }
                        }
                    }
                }
            }
        }

        protected void FemaleChooserCallback(string female)
        {
            SuperController.LogMessage(female + " selected.");
        }

        protected void MaleChooserCallback(string male)
        {
            SuperController.LogMessage(male + " selected.");
        }

        protected void DiagnosticWindowChooserCallback(string setting)
        {
            if (setting == "Show" && diagnosticsTextFieldOutput == false)
            {
                diagnostics = new JSONStorableString("Diagnostics Output", "");
                diagnosticsTextField = CreateTextField(diagnostics, true);
                diagnosticsTextField.height = 440f;

                diagnosticsTextFieldOutput = true;
            }
            else if (setting == "Hide" && diagnosticsTextFieldOutput == true)
            {
                RemoveTextField(diagnosticsTextField);

                diagnosticsTextFieldOutput = false;
            }
        }

        protected void SerialPortChooserCallback(string serialport)
        {
            SuperController.LogMessage(serialport + " selected.");
        }

        protected void ConnectToRobotCallback()
        {
            StartSerial();
        }

        protected void DisconnectFromRobotCallback()
        {
            StopSerial();
        }

        void StartSerial()
        {
            if (serialPortChooser.val != "None")
            {
                // Add extra characters for COM ports > 9
                if (serialPortChooser.val.Substring(0, 3) == "COM")
                {
                    if (serialPortChooser.val.Length != 4)
                    {
                        serialPortChooser.val = "\\\\.\\" + serialPortChooser.val;
                    }
                }

                // Open the serial connection
                serial = new SerialPort(serialPortChooser.val, Convert.ToInt32(baudChooser.val));
                serial.Open();
                serial.ReadTimeout = 10;

                if (serial != null && serial.IsOpen)
                {
                    SuperController.LogMessage("Connected to robot using serial port " + serialPortChooser.val + " at a baud rate of " + baudChooser.val);
                }
            }
        }

        void StopSerial()
        {
            if (serial != null && serial.IsOpen)
            {
                serial.Close();

                SuperController.LogMessage("Disconnected from robot using port " + serialPortChooser.val);
            }
        }
        private string StringToHex(string hexstring)
        {
            StringBuilder sb = new StringBuilder();
            foreach (char t in hexstring)
            {
                //Note: X for upper, x for lower case letters
                sb.Append(Convert.ToInt32(t).ToString("x") + " ");
            }
            return sb.ToString();
        }
        void SendCommandToRobot(float robotX, float robotY, float robotZ, float robotRY, float robotRZ)
        {
            if (protocolChooser.val == "T-Code v0.2")
            {
                // Formulate T-Code v0.2 command string
                string command = "L0" + GenerateTCode(robotX, 0.0f, 1.0f) + "\n";
                command += "L1" + GenerateTCode(robotY, 0.0f, 1.0f) + "\n";
                command += "L2" + GenerateTCode(robotZ, 0.0f, 1.0f) + "\n";
                command += "R1" + GenerateTCode(robotRY, 0.0f, 1.0f) + "\n";
                command += "R2" + GenerateTCode(robotRZ, 0.0f, 1.0f);

                // Send command to the robot
                if (serial != null && serial.IsOpen)
                {
                    serial.WriteLine(command);
                }

                if (diagnosticsTextFieldOutput)
                {
                    diagnostics.val += "\nT-Code v0.2:\n" + command;
                }
            }
            else if (protocolChooser.val == "RSM")
            {
                // RSM controlled by directly commanding servos through Pololu Maestro Servo Controller, 2 axes only
                float servo0f, servo1f;

                // Mix channels to send servo commands in the range 4000-8000 (quarter microseconds)
                // Note one servo inverted
                servo0f = 8000 - 4000 * robotX - 2000 * (robotRY - 0.5f);   // Left servo
                servo1f = 4000 + 4000 * robotX - 2000 * (robotRY - 0.5f);   // Right servo

                // Make sure servos stay within command range limit
                if (servo0f > 8000) servo0f = 8000;
                if (servo0f < 4000) servo0f = 4000;
                if (servo1f > 8000) servo1f = 8000;
                if (servo1f < 4000) servo1f = 4000;

                // Convert floats into unsigned integers
                UInt32 servo0, servo1;
                servo0 = Convert.ToUInt32(servo0f);
                servo1 = Convert.ToUInt32(servo1f);

                // Create 8-byte array to contain serial command
                byte[] serialBytes = new byte[8];
                serialBytes[0] = 0x84;                                    // Move command identifier
                serialBytes[1] = 0x00;                                    // Servo number (left servo - 0)
                serialBytes[2] = Convert.ToByte(servo0 & 0x7F);           // First 7 bits of 14-bit position command
                serialBytes[3] = Convert.ToByte((servo0 >> 7) & 0x7F);    // Second 7 bits of 14-bit position command
                serialBytes[4] = 0x84;                                    // Move command identifier
                serialBytes[5] = 0x01;                                    // Servo number (right servo - 1)
                serialBytes[6] = Convert.ToByte(servo1 & 0x7F);           // First 7 bits of 14-bit position command
                serialBytes[7] = Convert.ToByte((servo1 >> 7) & 0x7F);    // Second 7 bits of 14-bit position command

                // Send command to the robot
                if (serial != null && serial.IsOpen)
                {
                    serial.Write(serialBytes, 0, serialBytes.Length);
                }

                if (diagnosticsTextFieldOutput)
                {
                    diagnostics.val += "\nRSM:\n" + serialBytes[0].ToString("0") + " " + serialBytes[1].ToString("0") + " " + serialBytes[2].ToString("0") + " " + serialBytes[3].ToString("0") + "\n" + serialBytes[4].ToString("0") + " " + serialBytes[5].ToString("0") + " " + serialBytes[6].ToString("0") + " " + serialBytes[7].ToString("0");
                }
            }
        }

        string GenerateTCode(float input, float min, float max)
        {
            if (input > max) input = max;
            if (input < min) input = min;

            input = input * 1000;

            string output;

            if (input >= 999f)
            {
                output = "999";
            }
            else if (input >= 1f)
            {
                output = input.ToString("000");
            }
            else
            {
                output = "000";
            }

            return output;
        }

        protected void OnDestroy()
        {
            try
            {
                StopSerial();
            }
            catch (Exception e)
            {
                SuperController.LogError("Exception caught: " + e);
            }
        }
    }

    // Class for the females
    public class Female
    {
        public string name;
        public Vector3[] vagina;
        public Vector3[] mouth;
        public Vector3 vaginaVector;
        public Vector3 mouthVector;
        public string[] vaginaTriggers = { "LabiaTrigger", "VaginaTrigger", "DeepVaginaTrigger", "DeeperVaginaTrigger" };
        public string[] mouthTriggers = { "LipTrigger", "MouthTrigger", "ThroatTrigger" };
        public bool[] vaginaInsertionFlags;
        public bool[] mouthInsertionFlags;
        public bool[] vaginaTipInsertionFlags;
        public bool[] mouthTipInsertionFlags;
        public Rigidbody[] vaginaRigidbodies;
        public Rigidbody[] mouthRigidbodies;
        public List<Rigidbody[]> femaleRigidbodies;

        public Female(string female)
        {
            name = female;

            vagina = new Vector3[vaginaTriggers.Length];

            mouth = new Vector3[mouthTriggers.Length];

            vaginaRigidbodies = new Rigidbody[vaginaTriggers.Length];

            vaginaInsertionFlags = new bool[vaginaTriggers.Length];

            vaginaTipInsertionFlags = new bool[vaginaTriggers.Length];

            for (int i = 0; i < vaginaTriggers.Length; i++)
            {
                vaginaRigidbodies[i] = SuperController.singleton.GetAtomByUid(female).rigidbodies.First(rb => rb.name == vaginaTriggers[i]);

                vaginaInsertionFlags[i] = false;
            }

            mouthRigidbodies = new Rigidbody[mouthTriggers.Length];

            mouthInsertionFlags = new bool[mouthTriggers.Length];

            mouthTipInsertionFlags = new bool[mouthTriggers.Length];

            for (int i = 0; i < mouthTriggers.Length; i++)
            {
                mouthRigidbodies[i] = SuperController.singleton.GetAtomByUid(female).rigidbodies.First(rb => rb.name == mouthTriggers[i]);

                mouthInsertionFlags[i] = false;
            }
        }

        public void update()
        {
            for (int i = 0; i < vaginaTriggers.Length; i++)
            {
                vagina[i] = vaginaRigidbodies[i].position;
            }

            vaginaVector = vagina[0] - vagina[vaginaTriggers.Length - 1];
        
            for (int i = 0; i < mouthTriggers.Length; i++)
            {
                mouth[i] = mouthRigidbodies[i].position;
            }

            mouthVector = mouth[0] - mouth[mouthTriggers.Length - 1];
        }
    }

    // Class for the males
    public class Male
    {
        public string name;
        public Vector3[] penis;
        public Vector3 penisVector;
        public string[] penisNames = { "AutoColliderGen1Hard", "AutoColliderGen2Hard", "AutoColliderGen3aHard", "AutoColliderGen3bHard" };
        public GameObject[] penisColliders;
        public Rigidbody abdomenControl;
        public Vector3 abdomen;

        public Male(string male)
        {
            name = male;

            penis = new Vector3[penisNames.Length];

            penisColliders = new GameObject[penisNames.Length];

            for (int i = 0; i < penisNames.Length; i++)
            {
                foreach (Collider c in SuperController.singleton.GetAtomByUid(male).gameObject.GetComponentsInChildren<Collider>())
                {
                    if (c.gameObject.name == penisNames[i])
                    {
                        penisColliders[i] = c.gameObject;
                    }
                }
            }

            abdomenControl = new Rigidbody();

            abdomenControl = SuperController.singleton.GetAtomByUid(male).rigidbodies.First(rb => rb.name == "abdomenControl");
        }

        public void update()
        {
            for (int i = 0; i < penisNames.Length; i++)
            {
                penis[i] = penisColliders[i].transform.position;
            }

            penisVector = penis[penisNames.Length - 1] - penis[0];

            abdomen = abdomenControl.transform.position;
        }
    }
}
