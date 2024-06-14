inertia = """Ixx	24857.592
		Ixy	57407.636
		Ixz	10103.481
		Iyx	57407.636
		Iyy	3.695E+05
		Iyz	-3287.752
		Izx	10103.481
		Izy	-3287.752
		Izz	3.872E+05"""

expected = 'ixx="0.02686424" ixy="-0.000074925857" ixz="-0.0006725" iyy="0.10371" iyz="-0.000003479" izz="0.11096"'

values = expected.split()
names = [text[:3] for text in values]
inertia = inertia.replace('\t', '')

inertia = inertia.split('\n')

inertias = {}
for iner in inertia:
    inertias[iner[:3].lower()] = float(iner[3:])/(1000*100*100*100)

text = ''
for name in names:
    text += name + '="'
    number = str('{:.10f}'.format(inertias[name]))
    while number[-1] == '0':
        number = number[:-1]
    text += number + '" '

text = text[:-1]
print(text)