
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>

// Paramètres de simulation
const int pasTemps = 64;
const float vitesseMax = 6.28f;

// Seuils utilisés pour la logique floue
const float seuilTresProche = 1000.0f;
const float seuilProche = 400.0f;
const float seuilLoin = 150.0f;

// Valeurs des commandes d'orientation
const float tournerDroite = -0.4f;
const float tournerGauche = 0.4f;
const float allerToutDroit = 0.0f;

// Valeurs des commandes de vitesse
const float vitesseLente = 3.0f;
const float vitesseMoyenne = 4.5f;
const float vitesseRapide = 6.28f;


int main(int argc, char **argv) {

    // Initialisation du robot
    wb_robot_init();

    // Récupération des capteurs infrarouges
    WbDeviceTag capteurAvant = wb_robot_get_device("ps1");
    WbDeviceTag capteurDroit = wb_robot_get_device("ps2");
    WbDeviceTag capteurGauche1 = wb_robot_get_device("ps5");
    WbDeviceTag capteurGauche2 = wb_robot_get_device("ps6");

    // Activation des capteurs
    wb_distance_sensor_enable(capteurAvant, pasTemps);
    wb_distance_sensor_enable(capteurDroit, pasTemps);
    wb_distance_sensor_enable(capteurGauche1, pasTemps);
    wb_distance_sensor_enable(capteurGauche2, pasTemps);

    // Récupération des moteurs
    WbDeviceTag moteurGauche =
        wb_robot_get_device("left wheel motor");
    WbDeviceTag moteurDroit =
        wb_robot_get_device("right wheel motor");

    // Activation du contrôle des moteurs par la vitesse
    wb_motor_set_position(moteurGauche, INFINITY);
    wb_motor_set_position(moteurDroit, INFINITY);

    // Initialisation des vitesses à zéro
    wb_motor_set_velocity(moteurGauche, 0.0);
    wb_motor_set_velocity(moteurDroit, 0.0);

    // Affichage du démarrage
    printf("--- Demarrage du robot avec logique floue ---\n");
    printf("Objectif : suivre le mur gauche avec 3 regles.\n");

    // Boucle principale de simulation
    while (wb_robot_step(pasTemps) != -1) {

    }

    // Libération des ressources
    wb_robot_cleanup();

    return 0;
}

