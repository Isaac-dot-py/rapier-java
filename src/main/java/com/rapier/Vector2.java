package com.rapier;

/**
 * Represents a 2D vector with double precision.
 */
public class Vector2 {
    public double x;
    public double y;
    
    public Vector2() {
        this(0.0, 0.0);
    }
    
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public Vector2 add(Vector2 other) {
        return new Vector2(this.x + other.x, this.y + other.y);
    }
    
    public Vector2 subtract(Vector2 other) {
        return new Vector2(this.x - other.x, this.y - other.y);
    }
    
    public Vector2 scale(double scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }
    
    public double length() {
        return Math.sqrt(x * x + y * y);
    }
    
    public double lengthSquared() {
        return x * x + y * y;
    }
    
    public Vector2 normalize() {
        double len = length();
        if (len > 0) {
            return new Vector2(x / len, y / len);
        }
        return new Vector2(0, 0);
    }
    
    public double dot(Vector2 other) {
        return this.x * other.x + this.y * other.y;
    }
    
    @Override
    public String toString() {
        return String.format("Vector2(%.3f, %.3f)", x, y);
    }
}
