package com.rapier;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Utility class to load the native Rapier library.
 */
public class NativeLibraryLoader {
    private static File loadedLibraryFile = null;
    
    public static synchronized File loadLibrary() {
        if (loadedLibraryFile != null) {
            return loadedLibraryFile;
        }
        
        try {
            // Extract and load from resources
            String libName = getLibraryName();
            String resourcePath = "/native/" + libName;
            
            InputStream in = NativeLibraryLoader.class.getResourceAsStream(resourcePath);
            if (in == null) {
                throw new RuntimeException("Native library not found in resources: " + resourcePath);
            }
            
            // Create temp file
            File tempFile = File.createTempFile("rapier_java_ffi", getLibraryExtension());
            tempFile.deleteOnExit();
            
            // Copy to temp file
            try (OutputStream out = new FileOutputStream(tempFile)) {
                byte[] buffer = new byte[8192];
                int bytesRead;
                while ((bytesRead = in.read(buffer)) != -1) {
                    out.write(buffer, 0, bytesRead);
                }
            }
            in.close();
            
            loadedLibraryFile = tempFile;
            System.out.println("Native library extracted to: " + tempFile.getAbsolutePath());
            return loadedLibraryFile;
        } catch (Exception e) {
            throw new RuntimeException("Failed to load native library", e);
        }
    }
    
    private static String getLibraryName() {
        String os = System.getProperty("os.name").toLowerCase();
        if (os.contains("win")) {
            return "rapier_java_ffi.dll";
        } else if (os.contains("mac")) {
            return "librapier_java_ffi.dylib";
        } else {
            return "librapier_java_ffi.so";
        }
    }
    
    private static String getLibraryExtension() {
        String os = System.getProperty("os.name").toLowerCase();
        if (os.contains("win")) {
            return ".dll";
        } else if (os.contains("mac")) {
            return ".dylib";
        } else {
            return ".so";
        }
    }
}
