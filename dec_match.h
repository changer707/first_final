#ifndef DEC_MATCH_H
#define DEC_MATCH_H
#include "mulplayer.h"
#include <QThread>
    static bool closecam_n;
/*class WorkThread : public QThread{
    Q_OBJECT
public:
    WorkThread();


protected:

};*/

//std::string  dec_and_match(std::string video_path,MulPlayer*p,int channel,int mode,int num);
std::string  dec_and_match(std::string video_path,MulPlayer*p,int num);
//std::string  dec_and_match_2(std::string video_path,MulPlayer*p,int channel,int num);

/*namespace th_2{
        std::string  dec_and_match(std::string video_path,MulPlayer*p,int channel,int num);
}*/

//std::string  dec_and_match(std::string video_path,MulPlayer*p,int channel,int num);
#endif // DEC_MATCH_H
