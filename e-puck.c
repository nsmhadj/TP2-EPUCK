
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>

// Parametres du robot
const int pasTemps = 64;
const float vitesseMax = 6.28f;

// Les seuils pour la logique floue
const float seuilTresProche = 400.0f;
const float seuilProche = 300.0f;
const float seuilLoin = 100.0f;

// Les valeurs pour tourner
const float tournerDroite = -0.6f;
const float tournerGauche = 0.25f;
const float allerToutDroit = 0.0f;

// Les vitesses du robot
const float vitesseLente = 3.0f;
const float vitesseMoyenne = 4.5f;
const float vitesseRapide = 6.28f;


// Fonction pour savoir si le mur est proche
float est_proche(float valeur) {

    if (valeur >= seuilTresProche)
        return 1.0f;

    if (valeur <= seuilLoin)
        return 0.0f;

    return (valeur - seuilLoin) /
           (seuilTresProche - seuilLoin);
}


// Fonction pour savoir si le mur est loin
float est_loin(float valeur) {

    if (valeur <= seuilLoin)
        return 1.0f;

    if (valeur >= seuilProche)
        return 0.0f;

    return (seuilProche - valeur) /
           (seuilProche - seuilLoin);
}


// Fonction pour savoir si la distance est bonne
float est_bon(float valeur) {

    float difference = fabsf(valeur - seuilProche);
    float marge = seuilProche - seuilLoin;

    if (difference >= marge)
        return 0.0f;

    return 1.0f - (difference / marge);
}


// Calcul de l'orientation avec les regles floues
float controle_orientation(float murGauche, float obstacleDevant) {

    // Calcul des degres d'appartenance
    float murProche = est_proche(murGauche);
    float murLoin = est_loin(murGauche);
    float murBon = est_bon(murGauche);

    // On regarde si il y a un obstacle devant
    float dangerDevant = (obstacleDevant > 700.0f) ? 1.0f : 0.0f;

    // Regle 1 : mur proche ou obstacle devant, tourner a droite
    float alpha1 = fmaxf(murProche, dangerDevant);
    float c1 = tournerDroite;

    // Regle 2 : mur loin, tourner a gauche
    float alpha2 = murLoin;
    float c2 = tournerGauche;

    // Regle 3 : bonne distance et pas de danger, aller tout droit
    float alpha3 = fminf(murBon, 1.0f - dangerDevant);
    float c3 = allerToutDroit;

    // Calcul de la defuzzification
    float numerateur = (alpha1 * c1) +
                       (alpha2 * c2) +
                       (alpha3 * c3);

    float denominateur = alpha1 + alpha2 + alpha3;

    if (denominateur < 0.001f)
        return 0.0f;

    return numerateur / denominateur;
}


// Calcul de la vitesse avec les regles floues
float controle_vitesse(float murGauche, float obstacleDevant) {

    // On prend la valeur de danger la plus grande
    float dangerMax = fmaxf(murGauche, obstacleDevant);

    // Calcul des degres d'appartenance
    float dangerEleve = est_proche(dangerMax);
    float dangerMoyen = est_bon(dangerMax);
    float dangerFaible = est_loin(dangerMax);

    // Regle 1 : danger eleve, vitesse lente
    float alpha1 = dangerEleve;
    float c1 = vitesseLente;

    // Regle 2 : danger moyen, vitesse moyenne
    float alpha2 = dangerMoyen;
    float c2 = vitesseMoyenne;

    // Regle 3 : danger faible, vitesse rapide
    float alpha3 = dangerFaible;
    float c3 = vitesseRapide;

    // Calcul de la defuzzification
    float numerateur = (alpha1 * c1) +
                       (alpha2 * c2) +
                       (alpha3 * c3);

    float denominateur = alpha1 + alpha2 + alpha3;

    if (denominateur < 0.001f)
        return vitesseMoyenne;

    return numerateur / denominateur;
}


// Conversion de l'angle et la vitesse pour les deux roues
void convertir_vitesses(float angle, float vitesse, float *vg, float *vd) {

    // Si l'angle est positif on tourne a gauche
    if (angle > 0) {
        *vg = vitesse * (1.0f - angle);
        *vd = vitesse;
    }

    // Sinon on tourne a droite ou tout droit
    else {
        *vg = vitesse;
        *vd = vitesse * (1.0f + angle);
    }

    // On limite les vitesses des roues
    if (*vg > vitesseMax)
        *vg = vitesseMax;

    if (*vd > vitesseMax)
        *vd = vitesseMax;

    if (*vg < -vitesseMax)
        *vg = -vitesseMax;

    if (*vd < -vitesseMax)
        *vd = -vitesseMax;
}


// Fonction principale
int main(int argc, char **argv) {

    // Initialisation du robot
    wb_robot_init();

    // Recuperation des capteurs
    WbDeviceTag capteurAvant = wb_robot_get_device("ps1");
    WbDeviceTag capteurDroit = wb_robot_get_device("ps2");
    WbDeviceTag capteurGauche1 = wb_robot_get_device("ps5");
    WbDeviceTag capteurGauche2 = wb_robot_get_device("ps6");

    // Activation des capteurs
    wb_distance_sensor_enable(capteurAvant, pasTemps);
    wb_distance_sensor_enable(capteurDroit, pasTemps);
    wb_distance_sensor_enable(capteurGauche1, pasTemps);
    wb_distance_sensor_enable(capteurGauche2, pasTemps);

    // Recuperation des moteurs
    WbDeviceTag moteurGauche =
        wb_robot_get_device("left wheel motor");

    WbDeviceTag moteurDroit =
        wb_robot_get_device("right wheel motor");

    // On met les moteurs en mode vitesse
    wb_motor_set_position(moteurGauche, INFINITY);
    wb_motor_set_position(moteurDroit, INFINITY);

    // Au debut les roues sont a l'arret
    wb_motor_set_velocity(moteurGauche, 0.0);
    wb_motor_set_velocity(moteurDroit, 0.0);

    printf("--- Demarrage du robot avec logique floue ---\n");
    printf("Objectif : suivre le mur gauche avec 3 regles.\n");

    // Boucle principale
    while (wb_robot_step(pasTemps) != -1) {

        // Recuperation des valeurs des capteurs
        float valeurAvant = wb_distance_sensor_get_value(capteurAvant);
        float valeurGauche1 = wb_distance_sensor_get_value(capteurGauche1);
        float valeurGauche2 = wb_distance_sensor_get_value(capteurGauche2);

        // On prend la plus grande valeur a gauche
        float murGauche = fmaxf(valeurGauche1, valeurGauche2);

        // Valeur de l'obstacle devant
        float obstacleDevant = valeurAvant;

        // Calcul de l'orientation et de la vitesse
        float orientation = controle_orientation(murGauche, obstacleDevant);
        float vitesse = controle_vitesse(murGauche, obstacleDevant);

        // Calcul des vitesses des deux roues
        float vitesseGauche, vitesseDroite;

        convertir_vitesses(
            orientation,
            vitesse,
            &vitesseGauche,
            &vitesseDroite
        );

        // On envoie les vitesses aux moteurs
        wb_motor_set_velocity(moteurGauche, vitesseGauche);
        wb_motor_set_velocity(moteurDroit, vitesseDroite);

        // Affichage des valeurs dans la console
        printf("Mur: %4.0f | Devant: %4.0f || Angle: %+.2f | Vitesse: %.2f\n",
               murGauche, obstacleDevant, orientation, vitesse);
    }

    // Fin du programme
    wb_robot_cleanup();

    return 0;
}
