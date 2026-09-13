#include "../contributions/CloudDetector/CloudDetector.h"
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
// Offline replay of version-1 cloud debug records. Optional second argument overrides the
// mass-decline threshold (%/minute); external acquisition resets and motion gaps are retained.
int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: CloudDetectorReplay debug-log [decline-pct-per-minute]\n"; return 1; }
    std::ifstream input(argv[1]);
    if (!input) { std::cerr << "cannot open log\n"; return 1; }
    const float overrideRate = argc > 2 ? std::stof(argv[2]) : -1.f;
    CloudDetector detector;
    std::string line;
    std::cout << "time,tMs,state,mass,ensemble,declineRate,declineRatio,latched,exposure\n";
    while (std::getline(input, line)) {
        const auto pos = line.find("cloud: replay ");
        if (pos == std::string::npos) continue;
        std::istringstream words(line.substr(pos));
        std::map<std::string,std::string> f;
        std::string word;
        while (words >> word) { auto eq=word.find('='); if(eq!=std::string::npos)f[word.substr(0,eq)]=word.substr(eq+1); }
        if (f.at("v") != "1") { std::cerr << "unsupported replay version\n"; return 1; }
        auto number=[&](const char* key,double fallback=0.) { auto it=f.find(key);return it==f.end()?fallback:std::stod(it->second); };
        const auto before=detector.GetState();
        const auto event=f.at("event");
        if(event=="attach") {detector.SetEnabled(number("enabled")!=0);detector.SetSensitivityPct((int)number("sensitivity"));}
        else if(event=="reset")detector.Reset("replay");
        else if(event=="resume")detector.ResumeAfterMotion("replay");
        else if(event=="enabled")detector.SetEnabled(number("enabled")!=0);
        else if(event=="sensitivity")detector.SetSensitivityPct((int)number("sensitivity"));
        else if(event=="fault")detector.ReportFault("replay","recorded fault");
        else if(event=="feed") {
            SceneSample s;
            s.tMs=(int64_t)number("tMs");s.detected=number("detected")!=0;s.stableLock=number("stableLock")!=0;
            s.mass=(float)number("mass");s.snr=(float)number("snr");s.score=(float)number("score");s.features=(int)number("features");
            s.ensembleRatio=(float)number("ensembleRatio",-1);s.ensembleStars=(int)number("ensembleStars");s.ensembleTripRatio=(float)number("ensembleTripRatio",.78);
            s.brightCeil=(float)number("brightCeil",-1);s.brightExposureMs=(int)number("brightExposureMs");s.exposureMs=(int)number("exposureMs");
            s.gain=(int)number("gain",-1);s.bitDepth=(int)number("bitDepth",-1);s.frameW=(int)number("frameW",-1);s.frameH=(int)number("frameH",-1);
            s.roiX=(int)number("roiX",-1);s.roiY=(int)number("roiY",-1);s.roiW=(int)number("roiW",-1);s.roiH=(int)number("roiH",-1);
            s.sourceGen=(unsigned)number("sourceGen");s.mode=(int)number("mode");
            s.massDeclinePctPerMinute=overrideRate>=0?overrideRate:(float)number("massDeclinePctPerMinute");
            s.cloudConfigGeneration=(unsigned)number("cloudConfigGeneration");
            detector.Feed(s);
            const auto t=detector.GetTelemetry();
            std::cout<<line.substr(0,12)<<','<<s.tMs<<','<<(int)t.state<<','<<s.mass<<','<<t.ensembleRatio<<','<<t.massDeclineRate<<','<<t.massDeclineRatio<<','<<t.massDeclineLatched<<','<<s.exposureMs<<'\n';
        } else { std::cerr<<"unknown replay event: "<<event<<'\n';return 1; }
        const auto t=detector.GetTelemetry();
        if(before!=t.state)std::cerr<<line.substr(0,12)<<" "<<(int)before<<" -> "<<(int)t.state<<" rate="<<t.massDeclineRate<<" latch="<<t.massDeclineLatched<<'\n';
    }
}
