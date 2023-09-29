
class String():
    
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self):
        return self.x + self.y


string = String(10, 20)

string2 = String(100, 200)

x = string.x
y = string.y

x2 = string2.x
y2 = string2.y
print(x)
print(y)

print(x2)
print(y2)