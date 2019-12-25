for x in range(0,100):
    for y in range(0,100):
        for z in range (0,100):
            sum = 7*x + 2*y + z
            if ((sum == 90) and (y % 4 == 0 ) and (z % 4 == 0)):
                print(x, " ", y, " ", z)