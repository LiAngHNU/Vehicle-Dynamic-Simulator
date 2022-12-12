import struct
from _vs import *
outvals = []

cv_outarr = []


def output():
    global outvals
    return tuple(outvals)

class var:
    def __init__(self, name):
        self.varname = name
        self.id = getvar(name)
        self.val = getval(name)
        self.vstype = None
        self.vsunits = None

    def setvalue(self, newval):
        self.val = None
        return setval(self.varname, newval)

    def ident(self):
        return self.id

    def name(self):
        return self.varname

    def value(self):
        if (self.val == None):
            self.val = getval(self.varname)
        return self.val

    def type(self):
        if (self.vstype == None):
            self.vstype = gettype(self.varname)
        return self.vstype

    def units(self):
        if (self.vsunits == None):
            self.vsunits = getunits(self.varname).decode('ASCII')
        return self.vsunits

class tab():
    tabname = None
    def __init__(self, name):
        global tabname
        self.tabname = name
        tabname = name
        self.id = gettab(name)
        self.tabtype = None
        self.n_cols = None
        self.n_rows = None
        self.n_index = None

    def type(self):
        if (self.tabtype == None):
            self.tabtype = gettabtype(self.tabname)
        return self.tabtype

    def num_rows(self):
        if (self.n_rows == None):
            self.n_rows = gettabnx(self.tabname)
        return self.n_rows

    def num_cols(self):
        if (self.n_cols == None):
            self.n_cols = gettabny(self.tabname)
        return self.n_cols

    def num_index(self):
        if (self.n_index == None):
            self.n_index = gettabntab(self.tabname)
        return self.n_index


    class entry():
        def __init__(self, par1, par2, par3):
            global tabname
            #vs_tab.__init__(self)
            #self.ename = self.tabname
            #self.ename = mytabname
            self.ename = tabname
            self.currval = None
            self.currdefval = None
            self.par1 = par1
            self.par2 = par2
            self.par3 = par3

        def value(self):
            if (self.currval == None):
                self.currval = gettabval(self.ename, self.par1, self.par2, self.par3)
            return self.currval

        def defvalue(self):
            if (self.currdefval == None):
                self.currdefval = gettabdefval(self.ename, self.par1, self.par2, self.par3)
            return self.currdefval

        def setvalue(self, newval):
            # VS_TAB_STEP = 6
            if (gettabtype(self.ename) == 6):
                self.currval = None
                self.currval = settabval(self.ename, self.par1, self.par2, self.par3, newval)
                return self.currval
            if (self.currval == None):
                self.currval = gettabval(self.ename, self.par1, self.par2, self.par3)
            return self.currval


def collect_values(signal, outname, intab):
    global cv_outarr

    if (signal == "START"):
        outarr = []

    if (signal == "APPEND"):
        entry = []
        num = len(intab)
        if (num >= 2):
            entry.append(intab[0])
            entry.append(intab[1])
            cv_outarr.append(entry)


    if (signal == "CREATE_TABLE"):
        create_table(cv_outarr, 'LTARG', 'SPLINE_FLAT', 'm', 'm')

    #No output for this routine
    return 0.0


def create_table (out_arr, tablename, tabletype, funits, xunits):

    fileheadername = tablename+".vstb"
    filedataname = tablename+".vsb"
    filehdr = open(fileheadername, 'w')
    filehdr.write('{\n')
    filehdr.write('  \"VsTableGroup\" : {\n')
    filehdr.write('    \"FLabel\" : \"VALUE\",\n')
    filehdr.write('    \"FUnits\" : \"')
    filehdr.write(funits)
    filehdr.write('\",\n')
    filehdr.write('    \"Title\" : \"New Table\",\n')
    filehdr.write('    \"Type\" : \"')
    filehdr.write(tabletype)
    filehdr.write('\",\n')
    filehdr.write('    \"Version\" : 1,\n')
    filehdr.write('    \"XLabel\" : \"XLABEL\",\n')
    filehdr.write('    \"XUnits\" : \"')
    filehdr.write(xunits)
    filehdr.write('\",\n')
    filehdr.write('    \"Columns\" : [ {\n')
    filehdr.write('        \"Name\" : \"XLabel\",\n')
    filehdr.write('        \"Version\" : 1\n')
    filehdr.write('     }, {\n')
    filehdr.write('        \"Name\" : \"Value\",\n')
    filehdr.write('        \"Version\" : 1\n')
    filehdr.write('     } ]\n')
    filehdr.write('  }\n')
    filehdr.write('}\n')
    filehdr.close()

    filedata = open(filedataname, 'wb')
    versbytes = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    newByteArray = bytearray(versbytes)
    filedata.write(newByteArray)

    dataSize = [8, 0, 0, 0]
    newByteArray = bytearray(dataSize)
    filedata.write(newByteArray)

    numChans = [2, 0, 0, 0]
    newByteArray = bytearray(numChans)
    filedata.write(newByteArray)

    for entry in out_arr:
        da0 = bytearray(struct.pack("d", entry[0]))
        da1 = bytearray(struct.pack("d", entry[1]))
        filedata.write(da0)
        filedata.write(da1)



    filedata.close()



if __name__=="__main__":
    main()
