
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>

// Parametres du robot
const int pasTemps = 64;
const float vitesseMax = 6.28f;

// Les seuils pour la logique floue
const float seuilTresProche = 1000.0f;
const float seuilProche = 400.0f;
const float seuilLoin = 150.0f;

// Les valeurs pour tourner
const float tournerDroite = -0.4f;
const float tournerGauche = 0.4f;
const float allerToutDroit = 0.0f;

// Les vitesses du robot
const float vitesseLente = 3.0f;
const float vitesseMoyenne = 4.5f;
const float vitesseRapide = 6.28f;


// Fonction pour savoir si le mur est proche
float est_proche(float valeur) {

    // Si la valeur est grande le mur est tres proche
    if (valeur >= seuilTresProche)
        return 1.0f;

    // Si la valeur est petite le mur est loin
    if (valeur <= seuilLoin)
        return 0.0f;

    // Sinon on calcule le degre d'appartenance
    return (valeur - seuilLoin) /
           (seuilTresProche - seuilLoin);
}


// Fonction pour savoir si le mur est loin
float est_loin(float valeur) {

    // Si la valeur est petite le mur est loin
    if (valeur <= seuilLoin)
        return 1.0f;

    // Si la valeur depasse le seuil proche
    if (valeur >= seuilProche)
        return 0.0f;

    // Calcul du degre d'appartenance
    return (seuilProche - valeur) /
           (seuilProche - seuilLoin);
}


// Fonction pour verifier si la distance est bonne
float est_bon(float valeur) {

    // On calcule la difference avec la distance voulue
    float difference = fabsf(valeur - seuilProche);
    float marge = seuilProche - seuilLoin;

    // Si on est trop loin de la valeur voulue
    if (difference >= marge)
        return 0.0f;

    // Sinon on retourne le degre d'appartenance
    return 1.0f - (difference / marge);
}


// Fonction principale
int main(int argc, char **argv) {

    // On initialise le robot
    wb_robot_init();

    // Recuperation des capteurs
    WbDeviceTag capteurAvant = wb_robot_get_device("ps1");
    WbDeviceTag capteurDroit = wb_robot_get_device("ps2");
    WbDeviceTag capteurGauche1 = wb_robot_get_device("ps5");
    WbDeviceTag capteurGauche2 = wb_robot_get_device("ps6");

    // On active les capteurs
    wb_distance_sensor_enable(capteurAvant, pasTemps);
    wb_distance_sensor_enable(capteurDroit, pasTemps);
    wb_distance_sensor_enable(capteurGauche1, pasTemps);
    wb_distance_sensor_enable(capteurGauche2, pasTemps);

    // Recuperation des deux moteurs
    WbDeviceTag moteurGauche =
        wb_robot_get_device("left wheel motor");

    WbDeviceTag moteurDroit =
        wb_robot_get_device("right wheel motor");

    // On met les moteurs en mode vitesse
    wb_motor_set_position(moteurGauche, INFINITY);
    wb_motor_set_position(moteurDroit, INFINITY);

    // Au debut les deux roues sont a l'arret
    wb_motor_set_velocity(moteurGauche, 0.0);
    wb_motor_set_velocity(moteurDroit, 0.0);

    // Affichage dans la console
    printf("--- Demarrage du robot avec logique floue ---\n");
    printf("Objectif : suivre le mur gauche avec 3 regles.\n");

    // Boucle principale du robot
    while (wb_robot_step(pasTemps) != -1) {

    }

    // Fin du programme
    wb_robot_cleanup();

    return 0;
}
