"""Demo: receive a celestial target and generate alt-az axis commands."""

from datetime import datetime, timezone



from src.telescope import Observer, TelescopeSystem, from_catalog





def main():

    # Observer: Vadodara, Gujarat. Can change to position needed.

    observer = Observer(latitude_deg=22.31, longitude_deg=73.18)

    scope = TelescopeSystem(observer)



    target = from_catalog("vega")

    start = datetime(2026, 8, 30, 14, 30, 0, tzinfo=timezone.utc) # 20:00 IST



    # Cold start (encoders at 0,0) -> watch it slew, then track.

    # To start already pointed, call scope.initialize_to(target, start) first.

    scope.run(target, start, duration_s=120, dt=0.05, print_every=40)





if __name__ == "__main__":

    main()

