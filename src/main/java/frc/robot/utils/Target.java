
package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Core;
// import frc.robot.seafinder2.SF2Constants;
// import frc.robot.seafinder2.commands.StaticBackCommand;

public class Target {

    public enum Landmark {
        CARROT_PATCH, FEEDER_STATION
    }

    public enum Side {
        FRONT, BACK, LEFT, RIGHT
    }

    public static class Location {
        Landmark landmark;
        Side side;

        public Location(Landmark landmark, Side side) {
            this.landmark = landmark;
            this.side = side;
        }

        public Location(Landmark landmark) {
            this.landmark = landmark;
        }

        public Landmark getLandmark() {
            return landmark;
        }

        public Side getSide() {
            return side;
        }

        @Override
        public String toString() {
            return landmark.toString() + " " + side.toString();
        }
    }

    public static class TagRelativePose {
        int tagId;

        // NWU coordinate system
        double x;
        double y;
        double yaw;

        public TagRelativePose(int tagId, double x, double y, double yaw) {
            this.tagId = tagId;
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }

        public int getTagId() {
            return tagId;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getYaw() {
            return yaw;
        }

        public Pose2d getPose2d() {
            return new Pose2d(x, y, new Rotation2d(yaw));
        }

        public String toString() {
            return "Tag " + tagId + " at (" + x + ", " + y + ") with yaw " + yaw;
        }
    }

    Core core;

    Location location;

    TagRelativePose tagRelativePose;
    Command retractCommand;

    public Target(Core core) {
        this.core = core;
    }

    public void setLocation(Location location) {
        this.location = location;
        if (isValid()) {
            compute();
        }
    }

    public void compute() {
        init();
    }

    public boolean isValid() {
        return true;
    }

    public boolean isComputed() {
        return tagRelativePose != null;
    }

    public Location getLocation() {
        return location;
    }

    public TagRelativePose getTagRelativePose() {
        return tagRelativePose;
    }

    public int getTag() {
        return tagRelativePose.getTagId();
    }

    public Command getRetractCommand() {
        return retractCommand;
    }
    public String toString() {
        return "Target at " + location;
    }

    private void init() {
        final Optional<DriverStation.Alliance> driverStationAlliance = DriverStation.getAlliance();
        if (!driverStationAlliance.isPresent()) {
            throw new IllegalStateException("Driver station alliance not present");
        }
        final boolean isRed = driverStationAlliance.get() == Alliance.Red;

        int tagId = -1;
        double x = 0;
        double y = 0;
        double yaw = 0;

        this.tagRelativePose = new TagRelativePose(tagId, x, y, yaw);

    }

}
