package edu.wpi.first.note;

import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.json.simple.parser.JSONParser;
import org.photonvision.simulation.VisionTargetSim;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.CollectionType;
import com.fasterxml.jackson.databind.type.TypeFactory;

public class NoteLoader {
    /**
     * 
     */
    private static String readJsonFromFile() {
        try {

            URL path = NoteLoader.class.getResource("test.json");
            String json = Files.readString(Paths.get(path.toURI()));

            return new JSONParser().parse(json).toString();

        } catch (Exception e) {
            return "";
        }
    }

    /**
     * 
     */
    public static List<VisionTargetSim> getVisionTargets() {
        try {
            ObjectMapper objectMapper = new ObjectMapper();
            String jsonString = readJsonFromFile();

            CollectionType typeReference = TypeFactory.defaultInstance().constructCollectionType(List.class, Note.class);
            List<Note> notes = objectMapper.readValue(jsonString, typeReference);

            return new ArrayList<VisionTargetSim>();

        } catch (Exception e) {
            return new ArrayList<VisionTargetSim>();
        }
    }
}
