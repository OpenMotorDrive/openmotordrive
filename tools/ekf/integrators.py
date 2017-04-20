def rk(a,b,c,x_0,x_dot,dt):
    N = a.rows
    assert a.cols == N and len(b) == N and len(c) == N

    k = []

    for i in range(N):
        x_n = x_0
        for j in range(1,i):
            x_n += dt*a[i,j]*k[j]
        k.append(x_dot.xreplace(dict(zip(x_0, x_n))))

    x_n = x_0
    for i in range(N):
        x_n += dt*b[i]*k[i]

    return x_n

def rk3(x_0, x_dot, dt):
    a = Matrix([[0, 0, 0],
                [Rational(1,2),0,0],
                [-1,2,0]])
    b = Matrix([Rational(1,6), Rational(2,3), Rational(1,6)])
    c = Matrix([0, Rational(1,2),1])

    return rk(a,b,c,x_0,x_dot,dt)

def rk4(x_0, x_dot, dt):
    a = Matrix([[0, 0, 0, 0],
                [Rational(1,2), 0,0,0],
                [0,Rational(1,2),0,0],
                [0,0,1,0]])
    b = Matrix([Rational(1,6),Rational(1,3), Rational(1,3), Rational(1,6)])
    c = Matrix([0, Rational(1,2),Rational(1,2),1])

    return rk(a,b,c,x_0,x_dot,dt)

def rk438(x_0, x_dot, dt):
    a = Matrix([[0, 0, 0, 0],
                [Rational(1,3), 0,0,0],
                [-Rational(1,3),1,0,0],
                [1,-1,1,0]])
    b = Matrix([Rational(1,8),Rational(3,8), Rational(3,8), Rational(1,8)])
    c = Matrix([0, Rational(1,3),Rational(2,3),1])

    return rk(a,b,c,x_0,x_dot,dt)
