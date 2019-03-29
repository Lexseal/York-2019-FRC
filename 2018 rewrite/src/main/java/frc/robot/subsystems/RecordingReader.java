package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class RecordingReader {
	FileReader file;
	BufferedReader reader;
	ArrayList<double[]> vectors = new ArrayList<double[]>();
	int cutoffIdx = 0;
	
	public RecordingReader(String fileName) {
		double startTime = System.currentTimeMillis()/1000.0;
		
		String path = new String("/home/lvuser/recordings/");
		File folder = new File(path);
		if (!folder.exists()) {
			folder.mkdirs();
		}
		String pathToFile = new String(path+fileName+".csv");
		
		try {
			file = new FileReader(pathToFile);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		reader = new BufferedReader(file);
		try {
			String line =  null;
			while (true) {
				line = reader.readLine();
				if (line == null) {
					break;
				} else {
					String[] tokens = line.split(",");
					double[] doubleTokens = new double[tokens.length];
					for (int i = 0; i < tokens.length; i++) {
						doubleTokens[i] = Double.parseDouble(tokens[i]);
					}
					vectors.add(doubleTokens);
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		/*for (int i = 0; i < vectors.size(); i++) {
			double[] info = vectors.get(i);
			for (int n = 0; n < info.length; n++) {
				System.out.print(info[n]+" ");
			}
			System.out.println("");
		}*/
		
		System.out.println(System.currentTimeMillis()/1000.0-startTime);
	}

	public void resetCutoff() {
		cutoffIdx = 0;
	}

	public double getTotalTime() {
		return (vectors.size()-1)*0.005;
	}
	
	public double[] getVector(double targetTime) {
		/*int targetIdx = cutoffIdx;
		for (int i = cutoffIdx; i < vectors.size(); i++) {
			double[] line = vectors.get(i);
			double timeStamp = line[0];
			if (timeStamp <= targetTime) {
				targetIdx = i;
			} else {
				break;
			}
		}
		cutoffIdx = targetIdx;*/
		int targetIdx = (int)(targetTime/0.005);
		//System.out.println(targetIdx);
		return vectors.get(targetIdx);
	}
}
