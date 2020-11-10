# https://github.com/qpit/thorlabs_apt

import thorlabs_apt as apt
print(apt.list_available_devices())
upperTopSN = 27004949
upperBtmSN = 27004956
lowerTopSN = 27004948
lowerBtmSN =  27004958

upperTop = apt.Motor(upperTopSN)
upperBtm = apt.Motor(upperBtmSN)
lowerTop = apt.Motor(lowerTopSN)
lowerBtm = apt.Motor(lowerBtmSN)

upperTop.move_to(3)
upperBtm.move_to(3)
lowerTop.move_to(3)
lowerBtm.move_to(3)

upperTop.move_home(True)
upperTop.move_by(1)