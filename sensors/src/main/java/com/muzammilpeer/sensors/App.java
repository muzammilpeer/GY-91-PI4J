package com.muzammilpeer.sensors;


import com.muzammilpeer.sensors.gy91.GY91;

public class App {

    public static void main(String[] args) throws Exception {
        GY91 gy91 = new GY91();
        gy91.readDataFromSensors();
    }

}

