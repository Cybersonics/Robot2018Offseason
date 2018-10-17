package org.usfirst.frc.team103.unused;

import java.util.List;
import java.util.stream.Collectors;

import org.usfirst.frc.team103.unused.pixy.PixyBlock;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
	
	public static class CubeTarget {
		public final int x, y;
		public final int width, height;
		
		public CubeTarget(int x, int y, int width, int height) {
			this.x = x;
			this.y = y;
			this.width = width;
			this.height = height;
		}
	}
	
	public static CubeTarget findCube() {
		//return findCube(pixy.getBlocks());
		return null;
	}
	
	public static CubeTarget findCube(List<PixyBlock> blocks){
		
		//do the findCube
		
		SmartDashboard.putNumber("PixyCount", blocks.size());
    	for (int i = 0; i < 5; i++) {
    		SmartDashboard.putString("PixyBlock" + i, (i < blocks.size() ? blocks.get(i).toString() : ""));
    	}
		
		if (!blocks.isEmpty()) {
			List<PixyBlock> filteredBlocks = blocks
				.stream()
				.filter((PixyBlock b) -> b.signature == 2)
				//.filter((PixyBlock b) -> b.width * b.height > 500)
				.filter((PixyBlock b) -> {
					double ratio = (double) b.width / (double) b.height;
					return (ratio < 2.0) && (ratio > 0.5);
				})
				//.sorted((PixyBlock b1, PixyBlock b2) -> Math.abs(160 - b1.x) - Math.abs(160 - b2.x))
				//.sorted((PixyBlock b1, PixyBlock b2) -> b1.width * b1.height - b2.width * b2.height)
				.sorted((PixyBlock b1, PixyBlock b2) -> -((b1.y + b1.height / 2) - (b2.y + b2.height / 2)))
				.collect(Collectors.toList());
			if (!filteredBlocks.isEmpty()) {
				PixyBlock targetBlock = filteredBlocks.get(0);
				SmartDashboard.putString("PixyTarget", targetBlock.toString());
				return new CubeTarget(targetBlock.x, targetBlock.y, targetBlock.width, targetBlock.height);
			} else {
				SmartDashboard.putString("PixyTarget", "");
				return null;
			}
		} else {
			SmartDashboard.putString("PixyTarget", "");
			return null;
		}
	}

}
