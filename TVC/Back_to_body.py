#Author-Boopathi Sivakumar
#Description- Moves all the component bodies to root component

import adsk.core, adsk.fusion, traceback

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)

        # Get the root component of the active design.
        rootComp = design.rootComponent

        features_ = rootComp.features.removeFeatures

        # Set the design type to Parametric if not
        design.designType = 1

        # Build a list of occurrences 
        occs = []
        for occ in rootComp.occurrences:
            
            occs.append(occ)
            for occ_ in occ.childOccurrences:
                occs.append(occ_)

        occ: adsk.fusion.Occurrence
        dialouge = ui.messageBox('Do you want to move all the component bodies to Root Component?','Back to body',adsk.core.MessageBoxButtonTypes.YesNoButtonType)

        sourceBodies = adsk.core.ObjectCollection.create()
        if dialouge == adsk.core.DialogResults.DialogYes:
            for occ in occs:
                for body in occ.bRepBodies:
                    sourceBodies.add(body)
        else:
            return
        rootComp.features.cutPasteBodies.add(sourceBodies)

        #remove the empty componets using 
        for occ in occs:
            features_.add(occ)
        
        #Block this code if you want to capture the design history
        design.designType = 0

        ui.messageBox("All the component bodies are moved to root component")
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))