
#include <Arduino.h>
#include <Mouvement.h>
#include <Zumo32U4.h>
// Change next line with your type of display:
typedef Zumo32U4LCD Ecran;
// typedef Zumo32U4LCD Ecran;

class Affichage
{
public:
    void begin(){
        display = new(Ecran);
    }

    void afficher(Position pos);
private:
    /* data */
    Ecran *display;
};
