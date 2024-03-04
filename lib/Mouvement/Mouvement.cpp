#include <Mouvement.h>

Mouvement::Mouvement(Asservissement *motorD, Asservissement *motorG)
{
    motorD_ = motorD;
    motorG_ = motorG;
    resetCodeurD();
    resetCodeurG();
    x_pos = 0; y_pos = 0; theta_pos = 0;
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
    posD = lireCodeurD();
    posG = lireCodeurG();
    
    float deltaDist, ang;

    deltaDist = 0.5*((posD - pastPosD) + (posG - pastPosG));

    //Calcul de la valeur de l'angle parcouru
    ang = (((posD - pastPosD) - (posG - pastPosG))*1800.0*PERIMETRE_ROUE/(LARGEUR_ROBOT*M_PI*RESOLUTION_ROUE_CODEUSE));
    
    //Determination de la position sur le terrain en X, Y, Theta
    theta_pos +=  ang; 
    x_pos += deltaDist*cos((double)(theta_pos*M_PI/1800.0))*PERIMETRE_ROUE/RESOLUTION_ROUE_CODEUSE;
    y_pos += deltaDist*sin((double)(theta_pos*M_PI/1800.0))*PERIMETRE_ROUE/RESOLUTION_ROUE_CODEUSE;

    pastPosD = posD;
    pastPosG = posG;
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
    static float distEn1Sec = 184.5, coef = 1./distEn1Sec;
    static float waitTime = 1., sens = 1;
    // float dist = (distance * RESOLUTION_ROUE_CODEUSE) / PERIMETRE_ROUE;//En tick d'encodeurs
    switch (etat_mouvement)
    {
    case 0:
    {
        waitTime = distance * coef;
        if(distance < 0){sens = -1;}
        else{sens = 1;}
        write_PWMD((VIT_MAX/2) * sens);
        write_PWMG((VIT_MAX/2)  * sens);
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
        write_PWMG((VIT_MAX/2)  * sens);
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

