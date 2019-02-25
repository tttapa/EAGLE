#include <AlmostEqual.hpp>
#include <DroneParamsAndMatrices.hpp>
#include <FileLoader.hpp>
#include <LQRController.hpp>
#include <LeastSquares.hpp>  // inv
#include <PerfTimer.hpp>
#include <cassert>

using namespace std;

void DroneParamsAndMatrices::load(const filesystem::path &loadPath) {

    PerfTimer timer;

    /* Attitude */

    Aa_att = loadMatrix<Nx_att, Nx_att>(loadPath / "attitude" / "Aa");
    Ba_att = loadMatrix<Nx_att, Nu_att>(loadPath / "attitude" / "Ba");
    Ca_att = loadMatrix<Ny_att, Nx_att>(loadPath / "attitude" / "Ca");
    Da_att = loadMatrix<Ny_att, Nu_att>(loadPath / "attitude" / "Da");

    Ad_att = loadMatrix<Nx_att, Nx_att>(loadPath / "attitude" / "Ad");
    Bd_att = loadMatrix<Nx_att, Nu_att>(loadPath / "attitude" / "Bd");
    Cd_att = loadMatrix<Ny_att, Nx_att>(loadPath / "attitude" / "Cd");
    Dd_att = loadMatrix<Ny_att, Nu_att>(loadPath / "attitude" / "Dd");

    G_att = calculateG(Ad_att, Bd_att, Cd_att, Dd_att);

    Ad_att_r = getBlock<1, Nx_att, 1, Nx_att>(Ad_att);
    Bd_att_r = getBlock<1, Nx_att, 0, Nu_att>(Bd_att);
    Cd_att_r = getBlock<1, Ny_att, 1, Nx_att>(Cd_att);

    Ts_att = loadDouble(loadPath / "attitude" / "Ts");

    gamma_n = loadMatrix<3, 3>(loadPath / "attitude" / "gamma_n");
    gamma_u = loadMatrix<3, 3>(loadPath / "attitude" / "gamma_u");

    /* Altitude */

    Aa_alt = loadMatrix<Nx_alt, Nx_alt>(loadPath / "altitude" / "Aa");
    Ba_alt = loadMatrix<Nx_alt, Nu_alt>(loadPath / "altitude" / "Ba");
    Ca_alt = loadMatrix<Ny_alt, Nx_alt>(loadPath / "altitude" / "Ca");
    Da_alt = loadMatrix<Ny_alt, Nu_alt>(loadPath / "altitude" / "Da");

    Ad_alt = loadMatrix<Nx_alt, Nx_alt>(loadPath / "altitude" / "Ad");
    Bd_alt = loadMatrix<Nx_alt, Nu_alt>(loadPath / "altitude" / "Bd");
    Cd_alt = loadMatrix<Ny_alt, Nx_alt>(loadPath / "altitude" / "Cd");
    Dd_alt = loadMatrix<Ny_alt, Nu_alt>(loadPath / "altitude" / "Dd");

    G_alt = calculateG(Ad_alt, Bd_alt, Cd_alt, Dd_alt);

    Ts_alt = loadDouble(loadPath / "altitude" / "Ts");

    /* General */

    Id     = loadMatrix<3, 3>(loadPath / "I");
    Id_inv = loadMatrix<3, 3>(loadPath / "I_inv");
    k1     = loadDouble(loadPath / "k1");
    k2     = loadDouble(loadPath / "k2");

    m  = loadDouble(loadPath / "m");
    ct = loadDouble(loadPath / "ct");
    Dp = loadDouble(loadPath / "Dp");

    auto duration = timer.getDuration<chrono::microseconds>();
    cout << "Loaded Drone data files in " << duration << " µs." << endl;

    nh = sqrt((m * g) / (ct * rho * pow(Dp, 4) * Nm));
    uh = nh / k1;

    // TODO

    double Fzh = ct * rho * pow(Dp, 4) * pow(nh, 2) * Nm;
    double Fg  = -g * m;

    cout << "nh  = " << nh << endl;
    cout << "uh  = " << uh << endl;
    cout << "Fzh = " << Fzh << endl;
    cout << "Fg  = " << Fg << endl;
    cout << "Ts_alt  = " << setprecision(17) << Ts_alt << endl;

    cout << "Ts_alt / Ts_att = " << (Ts_alt / Ts_att) << endl;

    assert(isAlmostEqual(Id_inv, inv(Id), 1e-12));
}