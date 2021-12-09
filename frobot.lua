function test()
	lampadas()
	reset()
	g2()
	g1()
	ltable( {9,8,7} , 5)
view( 865.8 , 1386.4 , -1332.6 , 700 , 500 , -2700 , -0.1 , 0.9 , -0.4 )
	movz( -600  )
end
 
function  g1()
	posa(1000 , 140 , 80 , 120 )
	posi( 2,0)
	posaik( 150, 150, -450)
	posaik( 150,250, -150)
	posaik( 750,250, -150)
	--posa( 0,80,0,90)
	posa( 0 , 140 , 80 , 120 )

	posa(1000 , 140 , 80 , 120 )
	posi(0,90)
	posaik( 150,250, -150)
end
--g1()

function  g3()
	posa(1000 , 140 , 80 , 120 )
	posi( 2,0)
	posi(1,30)
	movz(-350)
	posaik( 450,250, -150)
end
--g3()

function reset()
movz(-150)
posa(-90 , 140 , 80 , 120 )

end
--reset()

function  g2()
	movz(-150)
	posi( 2,0)
	posi(1,30)

	posaik( 150,250, -150)
end

-- g2()

function lampadas()

posa( 110 , -20 , 80 , 70 )
movz(-110)
end

 


--posa( 90 , 140 , 80 , 120 ) 

function go()
posa( 10 , 110 , 30 , 50 )
 posa( 10 , 50 , 90 , 100 )
 posa( 10 , 110 , 31 , 50 )
 posa(20,22,90,50)
posa( 10 , 50 , 30 , 100 )
--posa(11,22,0)
end


--movz( -450 )
--movz( 300 )

for i=0,4 do
--go()
end

--view( 865.8 , 1386.4 , -332.6 , 700 , 500 , -700 , -0.1 , 0.9 , -0.4 )
 

function gr()

posadebug( 0 )
posadebug( 1 )

posaik( 210,190, -150)

posaik( 210,290, -450)

posaik( 100,90, -150)
posaik( 150,150, -450)
posaik( 450,150, -450)
posaik( 450,150, -150)
posaik(550,50, -450)

posaik( 750,250, -150)
posaik(  1050,250, -150)
posaik(  1050,250, -450)
posaik(  1050,150, -450)
end


