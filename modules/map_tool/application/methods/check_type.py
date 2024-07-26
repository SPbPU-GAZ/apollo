
def is_int(s):
    try: 
        int(s)
    except ValueError:
        return False
    else:
        return True

def is_float(s):
    try: 
        float(s)
    except ValueError:
        return False
    else:
        return True
