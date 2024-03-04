#include <Arduino.h>
#include <Motors.h>
#include <Asservissement.h>
  
#define TYPE_DEPLACEMENT_IMMOBILE 0         
#define TYPE_DEPLACEMENT_LIGNE_DROITE 1
#define TYPE_DEPLACEMENT_ROTATION 2
#define TYPE_MOUVEMENT_SUIVANT 10

#define RESOLUTION_ROUE_CODEUSE 909.7
#define DIAMETRE_ROUE (39./1000.)
#define PERIMETRE_ROUE (PI*DIAMETRE_ROUE)
#define LARGEUR_ROBOT (98./1000.)

#define TE_10MS 10
#define TE (TE_10MS*0.001)

struct Position {
  float x{}, y{}, a{};
};

typedef struct ligneDroite
{
    float distance;
}ligneDroite;

typedef struct rotation
{
    float degree;
}rotation;

typedef struct Ordre_deplacement
{
    char type;
    // Position consigne;
    ligneDroite line;
    rotation Rotation;
}Ordre_deplacement;

class Mouvement
{
public:
    Mouvement(Asservissement *motorD, Asservissement *motorG);
    ~Mouvement();
    void loop();//A mettre dans le loop
    
    bool nextActionPossible();//est ce qu'on peut passer au prochain mouvement?
    void ligneDroite(float distance, float dt);//distance en metre
    void rotation(float angle, float dt);//angle en radians

    float getDistance(const Position &p1, const Position &p2);

    Ordre_deplacement liste;//Position x y et theta en interne utilisées en ticks d'encodeurs !
    float next_action;
private:
    unsigned long start_time;

    float cmdD, cmdG;
    float posD, posG;//Position actuelle des deux moteurs en ticks d'encodeurs
    float pastPosD, pastPosG;
    Asservissement *motorD_;
    Asservissement *motorG_;

    float x_pos, y_pos, theta_pos;
    bool finMvtElem;

    void calcul();
    void Odometrie();
    bool TempsEchantionnage(uint16_t TIME);
};

