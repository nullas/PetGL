#include "elastic.h"


Elastic::Elastic()
{
}

void Elastic::initial(QWidget* _parent)
{
    parent = dynamic_cast<PetGL*>(_parent);
}

Q_EXPORT_PLUGIN2(elastic,Elastic)
