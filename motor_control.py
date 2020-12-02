# https://github.com/qpit/thorlabs_apt

import thorlabs_apt as apt
print("hi")
print(apt.list_available_devices())
upperTopSN = 27004949
upperBtmSN = 27004956
lowerTopSN = 27004948
lowerBtmSN = 27004958

upperTop = apt.Motor(upperTopSN)
upperBtm = apt.Motor(upperBtmSN)
lowerTop = apt.Motor(lowerTopSN)
lowerBtm = apt.Motor(lowerBtmSN)

upperTop.move_to(6)
upperBtm.move_to(6)
lowerTop.move_to(6)
lowerBtm.move_to(6)

# upperTop.move_home(True)
# upperTop.move_by(1)

#For home = 3
upperTopStart = 5 #49
upperBtmStart = 5.50006 #56
lowerTopStart = 6.70003 #48
lowerBtmStart = 5.35 #58
