from minerva_bot import MinervaBot

if __name__ == '__main__':
    M = MinervaBot()
    M.login()
    M.goToQuickAdd()
    M.register()