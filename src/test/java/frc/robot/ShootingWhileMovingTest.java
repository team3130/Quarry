package frc.robot;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.Shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;


public class ShootingWhileMovingTest {
        static Shooter shooter;

    @BeforeAll
    public static void initialize(){
        
    }

    @Test
    public void martyr() {
        shooter.getInterPolVel();
    }
}