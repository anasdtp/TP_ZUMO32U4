#include "Mouvement.h"

float acc_max = 0.5, v_max = 0.5, dcc_max = 0.5;
float ta = v_max / acc_max;
float d_max = v_max * v_max / acc_max;
float distance_;
FuncType Mouvement::fnVitesse2(float distance)
{
    distance_ = distance;
    if (distance < d_max)
    {
        return static_cast<FuncType>([](float time) -> float
        {
            float time_lim = sqrt(distance_ / acc_max);
            if (time < time_lim)
                return acc_max * time;
            else
                return v_max - acc_max * (time - time_lim);
        });
    }
    else
    {
        return static_cast<FuncType>([](float time) -> float
        {
            float tc = (distance_ - d_max) / v_max;
            if (time < ta)
                return acc_max * time;
            else if (time >= ta && time <= tc + ta)
                return v_max;
            else
                return v_max - acc_max * (time - ta - tc);
        });
    }
}

float Mouvement::trajectoire(float time, FuncType velocityProfile)
{
    float trajectory = 0.0;
    for (float t = 0.0; t <= time; t += time)
    {
        trajectory += velocityProfile(t) * time;
    }

    return trajectory;
}

Mouvement::Mouvement(Asservissement *motorD, Asservissement *motorG)
{
    motorD_ = motorD;
    motorG_ = motorG;
    resetCodeurD();
    resetCodeurG();
    pos_ = (Position){0,0,0};
    // capteur = new(Capteurs);
    finMvtElem = false;
    next_action = false;
    liste.type = TYPE_MOUVEMENT_SUIVANT;
    start_time = millis();
}

Mouvement::~Mouvement()
{
    // delete motorD_;
    // delete motorG_;
}





void Mouvement::loop()
{
    if (TempsEchantionnage(TE_10MS))
    {
        calcul();
    }
}

void Mouvement::Odometrie(){
    static float deltaDist, ang, init = true;
    if(init){init = false; resetCodeurD(); resetCodeurG();}

    posD = lireCodeurD()*1.;
    posG = lireCodeurG()*1.;

    deltaDist = 0.5*((posD - pastPosD) + (posG - pastPosG));

    //Calcul de la valeur de l'angle parcouru
    // ang = (((posD - pastPosD) - (posG - pastPosG))*PERIMETRE_ROUE*1800.0/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE));
    ang = ((posD - pastPosD) - (posG - pastPosG))*PI/1800.0;
    
    //Determination de la position sur le terrain en X, Y, Theta
    pos_.theta +=  (ang);  //en radians
    pos_.x += deltaDist*cos((double)(pos_.theta))*PERIMETRE_ROUE/RESOLUTION_ROUE_CODEUSE; //En metres
    pos_.y += deltaDist*sin((double)(pos_.theta))*PERIMETRE_ROUE/RESOLUTION_ROUE_CODEUSE; //En metres

    pastPosD = posD;
    pastPosG = posG;
    // Serial.println(pos_.x);
    // Serial.println(pos_.y);
    // Serial.println(pos_.theta*180./PI);//EN de
    // Serial.println("");
}

void Mouvement::calcul(){
    float dt = (millis() - start_time) * 0.001f;//En seconde

    Odometrie();

    switch (liste.type)
    {
    case (TYPE_DEPLACEMENT_IMMOBILE):
    {
        // posD = lireCodeurD();
        // posG = lireCodeurG();
        cmdD = motorD_->AsserMot(0, posD, dt);
        cmdG = motorG_->AsserMot(0, posG, dt);
        write_PWMD(cmdD);
        write_PWMG(cmdG);
        start_time = millis(); 
    }break;
    case (TYPE_DEPLACEMENT_LIGNE_DROITE):
    {
        ligneDroite(liste.line.distance, dt);
        if (finMvtElem)
        {
            finMvtElem = false;
            Serial.println("finMvtElem");
            liste.type = (TYPE_MOUVEMENT_SUIVANT);
            // etat_automate_depl = INITIALISATION;

            next_action = true;
        }
    }break;
    case (TYPE_DEPLACEMENT_ROTATION):
    {
        rotation(liste.Rotation.degree, dt);
        if (finMvtElem)
        {
            liste.type = (TYPE_MOUVEMENT_SUIVANT);
            // etat_automate_depl = INITIALISATION;
            finMvtElem = false;
            next_action = true;
        }
        
    }break;
    case (TYPE_DEPLACEMENT_SUIVIE_LIGNE):
    {
        int vitesse = 1000;
        ValeurCapteurs capteurs = capt.readSensors();
        if (capteurs.LigneAvant > 0 || capteurs.LigneDroit > 0 || capteurs.LigneGauche > 0){
            vitesse = 0;
        }

        cmdD = motorD_->AsserMot(posD+vitesse, posD, dt);
        cmdG = motorG_->AsserMot(posG+vitesse, posG, dt);

        write_PWMD(cmdD);
        write_PWMG(cmdG);
        
    }break;
    default:
    {}break;
    }

    if (liste.type == TYPE_MOUVEMENT_SUIVANT)
    {

            // Remise a zero des variables
            // etat_automate_depl = INITIALISATION;
            // On reset la position du robot pour la prochaine action
            resetCodeurD();
            resetCodeurG();
            start_time = millis();
            // on prévoit de s'asservir sur la position atteinte
            liste.type = TYPE_DEPLACEMENT_IMMOBILE;
    }
}

float Mouvement::getDistance(const Position &p1, const Position &p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


bool Mouvement::nextActionPossible(){
    if(next_action){
        next_action = false;
        return true;
    }
    return false;
}

void Mouvement::ligneDroite(float distance, float dt){//En mm
    static char etat_mouvement = 0; 
    static float cmdD = 0., cmdG = 0.;
    static Position targetPos;
    // float dist = (distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE;//En tick d'encodeurs
    switch (etat_mouvement)
    {
    case 0:
    {
        traj = fnVitesse2(distance);

        targetPos.theta = pos_.theta;
        targetPos.x = pos_.x + distance*cos(targetPos.theta);
        targetPos.y = pos_.y + distance*sin(targetPos.theta);

        start_time = millis();
        etat_mouvement = 1;
    }
    break;
    case 1:
    {
        // Generate trajectory based on desired position
        float trajectory = trajectoire(dt, traj);
        float error = trajectory + getDistance(pos_, targetPos);

        cmdD = motorD_->AsserMot(error, posD, dt);
        cmdG = motorG_->AsserMot(error, posG, dt);
    }
    break;
    case 2:
    {
        
        finMvtElem = true;
        etat_mouvement = 0;
    }
    break;
    default:
        break;
    }

    write_PWMD(cmdD);
    write_PWMG(cmdG);
}

void Mouvement::rotation(float angle, float dt){//En degres
    static char etat_mouvement = 0; 
    static float degreeEn1Sec = 135., coef = 1./degreeEn1Sec;
    static float waitTime = 1., sens = 1;
    // angle = LARGEUR_ROBOT * PI * RESOLUTION_ROUE_CODEUSE * angle / (360 * PERIMETRE_ROUE)//En tick d'encodeurs
    switch (etat_mouvement)
    {
    case 0:
    {
        waitTime = angle * coef;
        if(angle < 0){sens = -1;}
        else{sens = 1;}
        write_PWMD((-VIT_MAX/2) * sens);
        write_PWMG(( VIT_MAX/2) * sens);
        start_time = millis();
        etat_mouvement = 1;
    }
    break;
    case 1:
    {
        if(dt>waitTime){
            etat_mouvement = 2;
        }
    }
    break;
    case 2:
    {
        write_PWMD(0);
        write_PWMG(0);
        finMvtElem = true;
        etat_mouvement = 0;
    }
    break;
    default:
        break;
    }
}

Position Mouvement::getPosition(){
    return pos_;
}

bool Mouvement::TempsEchantionnage(uint16_t TIME)
{
    static uint32_t LastMscount = millis();
    if ((millis() - LastMscount) >= TIME)
    {
        LastMscount = millis();
        return true;
    }
    else
    {
        return false;
    }   
}

