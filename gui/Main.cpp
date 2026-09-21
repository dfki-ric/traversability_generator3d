#include <QApplication>
#include <execinfo.h> //gcc only
#include "TraversabilityGenerator3dGui.hpp"
#include <base-logging/Logging.hpp>


class Application : public QApplication {
public:
    Application(int& argc, char** argv) : QApplication(argc, argv) {}
    
    virtual bool notify(QObject *receiver, QEvent *e) 
    {
        try 
        {
            return QApplication::notify(receiver, e);
        } catch (std::exception &ex) 
        {
            LOG_ERROR_S << "CAUGHT exception in qt event loop:\n" << ex.what();
            const int traceLen = 40;
            void *symbols[traceLen];
            const size_t size = backtrace(symbols, traceLen);
            backtrace_symbols_fd(symbols, size, STDERR_FILENO);

        } catch (...) {
            LOG_ERROR_S << "CAUGHT unknown exception in qt event loop";
        }        
         return false;
     }
};

int main(int argc, char** argv)
{
    // The GUI process hosts 16+ llvmpipe software-render threads next to the
    // OpenMP team of the map expansion. With OpenMP's default ACTIVE wait
    // policy every worker spin-waits at each wave barrier and fights the
    // render threads for cores — measured ~4x slower expansion under desktop
    // load, recovered by passive waiting. Must be set before the first
    // parallel region; overwrite=0 keeps user overrides working.
    setenv("OMP_WAIT_POLICY", "passive", 0);
    // Cap Mesa llvmpipe's rasterizer pool (default: one thread per logical
    // CPU). 16 render threads competing with the OpenMP expansion team slowed
    // map generation ~7x in this GUI; 4 is plenty for a mostly static debug
    // view. No effect when real GPU drivers are used instead of llvmpipe.
    setenv("LP_NUM_THREADS", "4", 0);

    Application app(argc, argv);
    TraversabilityGenerator3dGui gui(argc, argv);
    gui.show();
    app.exec(); 
    return 0;
}
  
