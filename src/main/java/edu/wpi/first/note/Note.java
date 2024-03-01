package edu.wpi.first.note;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;

public class Note {
    @JsonProperty("id")
    private int id;
    // private Pose2d pose;

    /**
     * 
     */
    public Note(int id) {
        this.id = id;
        // this.pose = pose;
    }
}
